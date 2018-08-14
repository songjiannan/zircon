// Copyright 2018 The Fuchsia Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <ddk/protocol/platform-defs.h>

#include "qemu-test.h"

static const pbus_dev_t child_1_kids[] = {
    {
    },
    {
    },
};

static const pbus_dev_t parent_kids[] = {
    {
        .children = child_1_kids,
        .child_count = countof(child_1_kids),
    },
};

const pbus_dev_t test_dev = {
    .name = "qemu-test-parent",
    .vid = PDEV_VID_QEMU,
    .pid = PDEV_PID_QEMU,
    .did = PDEV_DID_QEMU_TEST_PARENT,
    .children = parent_kids,
    .child_count = countof(parent_kids),
};

zx_status_t qemu_test_init(platform_bus_protocol_t* pbus) {
    return pbus_device_add(pbus, &test_dev, 0);
}
