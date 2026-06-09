/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2021 Rockchip Electronics Co., Ltd
 */
#ifndef __SOC_ROCKCHIP_IOMMU_H
#define __SOC_ROCKCHIP_IOMMU_H

#include <linux/device.h>
#include <linux/iommu.h>

struct device;

#if IS_REACHABLE(CONFIG_ROCKCHIP_IOMMU)
int rockchip_iommu_enable(struct device *dev);
int rockchip_iommu_disable(struct device *dev);

static inline bool rockchip_iommu_is_enabled(struct device *dev)
{
	/* If the device has an IOMMU group, IOMMU is enabled */
	struct iommu_group *group = iommu_group_get(dev);
	if (group) {
		iommu_group_put(group);
		return true;
	}
	return false;
}

#else
static inline int rockchip_iommu_enable(struct device *dev)
{
	return -ENODEV;
}
static inline int rockchip_iommu_disable(struct device *dev)
{
	return -ENODEV;
}
#endif

#endif
