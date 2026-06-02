#include <stdio.h>
#include <string.h>
#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;

int main(int argc, char* argv[]) {
    if (argc != 3) {
        fprintf(stderr, "Usage: %s <current_ip> <new_ip>\n", argv[0]);
        fprintf(stderr, "Example: %s 192.168.11.2 192.168.11.10\n", argv[0]);
        return -1;
    }

    const char* current_ip = argv[1];
    const char* new_ip     = argv[2];

    auto driver_result = createLidarDriver();
    if (!driver_result) {
        fprintf(stderr, "Failed to create driver\n");
        return -1;
    }
    ILidarDriver* driver = *driver_result;

    // Connect via UDP
    auto channel = createUdpChannel(current_ip, 8089);
    if (!channel) {
        fprintf(stderr, "Failed to create UDP channel\n");
        delete driver;
        return -1;
    }

    auto res = driver->connect(*channel);
    if (SL_IS_FAIL(res)) {
        fprintf(stderr, "Failed to connect to lidar at %s: %x\n", current_ip, res);
        delete driver;
        return -1;
    }
    printf("Connected to lidar at %s\n", current_ip);

    // Parse new IP into 4 octets
    sl_lidar_ip_conf_t ipConf;
    memset(&ipConf, 0, sizeof(ipConf));

    unsigned int o1, o2, o3, o4;
    if (sscanf(new_ip, "%u.%u.%u.%u", &o1, &o2, &o3, &o4) != 4) {
        fprintf(stderr, "Invalid IP address format: %s\n", new_ip);
        delete driver;
        return -1;
    }

    ipConf.ip_addr[0] = o1;
    ipConf.ip_addr[1] = o2;
    ipConf.ip_addr[2] = o3;
    ipConf.ip_addr[3] = o4;

    ipConf.net_mask[0] = 255;
    ipConf.net_mask[1] = 255;
    ipConf.net_mask[2] = 255;
    ipConf.net_mask[3] = 0;

    // Gateway: same subnet, ending in .1
    ipConf.gw[0] = o1;
    ipConf.gw[1] = o2;
    ipConf.gw[2] = o3;
    ipConf.gw[3] = 1;

    res = driver->setLidarIpConf(ipConf);
    if (SL_IS_FAIL(res)) {
        fprintf(stderr, "Failed to set IP config: %x\n", res);
        delete driver;
        return -1;
    }
    printf("IP config set to %s. Rebooting lidar...\n", new_ip);

    driver->reset();
    delete driver;

    printf("Done. Lidar will come back up on %s\n", new_ip);
    return 0;
}
