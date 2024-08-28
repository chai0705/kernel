echo [boot.cmd] run boot.cmd scripts ...;

if test -e ${devtype} ${devnum}:${distro_bootpart} /uEnv/uEnv.txt; then

    echo [boot.cmd] load ${devtype} ${devnum}:${distro_bootpart} ${env_addr_r} /uEnv/uEnv.txt ...;
    load ${devtype} ${devnum}:${distro_bootpart} ${env_addr_r} /uEnv/uEnv.txt;

    echo [boot.cmd] Importing environment from ${devtype} ...
    env import -t ${env_addr_r} 0x8000

    setenv bootargs ${bootargs} root=/dev/mmcblk${devnum}p3 ${cmdline}
    printenv bootargs

    echo [boot.cmd] load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd-${uname_r} ...
    load ${devtype} ${devnum}:${distro_bootpart} ${ramdisk_addr_r} /initrd.img-${uname_r}

    echo [boot.cmd] loading ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image-${uname_r} ...
    load ${devtype} ${devnum}:${distro_bootpart} ${kernel_addr_r} /Image-${uname_r}

    echo [boot.cmd] loading default rk-kernel.dtb
    load ${devtype} ${devnum}:${distro_bootpart} ${fdt_addr_r} /rk-kernel.dtb

    fdt addr  ${fdt_addr_r}
    fdt set /chosen bootargs

    echo [boot.cmd] dtoverlay from /uEnv/uEnv.txt
    setenv dev_bootpart ${devnum}:${distro_bootpart}
    dtfile ${fdt_addr_r} ${fdt_over_addr}  /uEnv/uEnv.txt ${env_addr_r}

    echo [boot.cmd] [${devtype} ${devnum}:${distro_bootpart}] ...
    echo [boot.cmd] [booti] ...
    booti ${kernel_addr_r} ${ramdisk_addr_r} ${fdt_addr_r}
fi

echo [boot.cmd] run boot.cmd scripts failed ...;
