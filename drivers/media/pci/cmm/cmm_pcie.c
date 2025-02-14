#include <linux/init.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#define DEVICE_VENDOR_ID 0x10EE  // вставил нужный Vendor ID
#define DEVICE_PRODUCT_ID 0x7011 // вставил нужный Product ID

#define DEVICE_NAME "pcie_cmm_ctrl"
#define CLASS_NAME "pcie_cmm"

static struct pci_device_id pcie_cmm_controller_ids[] = {
    {PCI_DEVICE(DEVICE_VENDOR_ID, DEVICE_PRODUCT_ID), 0},
    {
        0,
    }};

MODULE_DEVICE_TABLE(pci, pcie_cmm_controller_ids);

static struct pcie_cmm_driver_priv {
    struct pci_dev *pdev;
    void __iomem *bar0; // Pointer of the first bar
    resource_size_t bar0_size;
    struct cdev cdev;
    dev_t devno;
};

static struct class *pcie_cmm_class;

static int device_cmm_open(struct inode *inode, struct file *file) {
    struct pcie_cmm_driver_priv *dev = container_of(inode->i_cdev, struct pcie_cmm_driver_priv, cdev);
    file->private_data = dev;
    pr_info("Device opened\n");
    return 0;   
}

static ssize_t device_cmm_read(struct file *file, char __user *buf, size_t count, loff_t *pos) {
    struct pcie_cmm_driver_priv *dev = file->private_data;
    unsigned int val;

    val = ioread32(dev->bar0 + *pos);

    if( copy_to_user(buf, &val, sizeof(val))) {
        return -EFAULT;
    }

    *pos += sizeof(val);
    return sizeof(val);
}


static ssize_t device_cmm_write(struct file *file, const char __user *buf, size_t count, loff_t *pos) {
    struct pcie_cmm_driver_priv *dev = file->private_data;
    unsigned int val;

    if( copy_from_user(&val, buf, sizeof(val))) {
        return -EFAULT;
    }

    iowrite32(val, dev->bar0 + *pos);

    *pos += sizeof(val);
    return sizeof(val);
}

static long device_cmm_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct pcie_cmm_driver_priv *dev = file->private_data;

    // TODO: Added cmd for ioctl
    return 0;
}

static const struct file_operations fops = {
    .owner = THIS_MODULE,
    .open = device_cmm_open,
    .read = device_cmm_read,
    .write = device_cmm_write,
    .unlocked_ioctl = device_cmm_ioctl,
};

static int pcie_cmm_controller_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
    struct pcie_cmm_driver_priv *drv_priv;
    int ret;

    drv_priv = kzalloc(sizeof(*drv_priv), GFP_KERNEL);

    if(!drv_priv) {
        dev_err(&pdev->dev,"Could not kzalloc()ate memory.\n");
        return -ENOMEM;
    }

    dev_info(&pdev->dev, "Vendor ID: 0x%04X\n", pdev->vendor);
    dev_info(&pdev->dev, "Device ID: 0x%04X\n", pdev->device);
    dev_info(&pdev->dev, "Subsystem Vendor ID: 0x%04X\n", pdev->subsystem_device);
    
    ret = pci_enable_device(pdev);
    if (ret) {
        dev_err(&pdev->dev, "Failed to enabled PCI device\n");
        return ret;
    }
    drv_priv->pdev = pdev;
    
    ret = pci_request_regions(pdev, "pcie_cmm_driver");
    if (ret) {
        dev_err(&pdev->dev, "Failed to request PCI regions\n");
        goto disable_device;
    }

    drv_priv->bar0 = pci_iomap(pdev, 0, 0);
    if (!drv_priv->bar0) {
        dev_err(&pdev->dev, "Failed to map BAR0\n");
        ret = -ENOMEM;
        goto release_regions;
    }

    drv_priv->bar0_size = pci_resource_len(pdev, 0);
    dev_info(&pdev->dev, "BAR0 mapped at %p, size: %lld bytes\n", drv_priv->bar0, drv_priv->bar0_size);

    ret = alloc_chrdev_region(&drv_priv->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to allocate device number\n");
        goto unmap_bar;
    }

    cdev_init(&drv_priv->cdev, &fops);
    drv_priv->cdev.owner = THIS_MODULE;
    ret = cdev_add(&drv_priv->cdev, drv_priv->devno, 1);
    if (ret < 0) {
        dev_err(&pdev->dev, "Failed to add cdev\n");
        goto unregister_chrdev;
    }

    device_create(pcie_cmm_class, NULL, drv_priv->devno, NULL, DEVICE_NAME);

    pci_set_drvdata(pdev, drv_priv);

    dev_info(&pdev->dev, "PCIe CMM Controller driver loaded\n");
    return 0;

unregister_chrdev:
    unregister_chrdev_region(drv_priv->devno, 1);
unmap_bar:
    pci_iounmap(pdev, drv_priv->bar0);
release_regions:
    pci_release_regions(pdev);
disable_device:
    pci_disable_device(pdev);
    return ret;
}

static void pcie_cmm_controller_remove(struct pci_dev *pdev)
{
    struct pcie_cmm_driver_priv *dev = pci_get_drvdata(pdev);

    // Удаляем устройство из /dev
    device_destroy(pcie_cmm_class, dev->devno);

    // Удаляем символьное устройство
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->devno, 1);

    // Освобождаем ресурсы
    pci_iounmap(pdev, dev->bar0);
    pci_release_regions(pdev);
    pci_disable_device(pdev);

    dev_info(&pdev->dev, "PCIe CMM Controller driver unloaded\n");
}

static struct pci_driver pci_cmm_controller_driver = {
    .name = DEVICE_NAME,
    .id_table = pcie_cmm_controller_ids,
    .probe = pcie_cmm_controller_probe,
    .remove = pcie_cmm_controller_remove,
};

static int __init pcie_cmm_controller_init(void) {
    pcie_cmm_class = class_create(CLASS_NAME);
    if (IS_ERR(pcie_cmm_class)) {
        pr_err("Failed to create class\n");
        return PTR_ERR(pcie_cmm_class);
    }

    return pci_register_driver(&pci_cmm_controller_driver);
}

static void __exit pcie_cmm_controller_exit(void)
{
    pci_unregister_driver(&pci_cmm_controller_driver);
    class_destroy(pcie_cmm_class);
}

module_init(pcie_cmm_controller_init);
module_exit(pcie_cmm_controller_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alex");
MODULE_DESCRIPTION("PCIe CMM Controller Driver");