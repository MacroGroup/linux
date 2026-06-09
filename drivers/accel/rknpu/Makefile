rknpu-y += rknpu_drv.o
rknpu-y += rknpu_gem.o
rknpu-y += rknpu_fence.o
rknpu-y += rknpu_job.o
rknpu-y += rknpu_reset.o
rknpu-y += rknpu_iommu.o
rknpu-y += rknpu_debugger.o
rknpu-y += rknpu_devfreq.o

# 


# rknpu-y += rknpu_mm.o
# rockchip-rknpu-y += rknpu_mem.o

ccflags-y += -I$(src)/
ccflags-y += -I$(src)/compat
ccflags-y += -DCONFIG_ROCKCHIP_RKNPU_DRM_GEM
ccflags-y += -DCONFIG_ROCKCHIP_RKNPU_FENCE
ccflags-y += -DCONFIG_ROCKCHIP_RKNPU_DEBUG_FS

obj-$(CONFIG_ROCKCHIP_RKNPU) += rknpu.o