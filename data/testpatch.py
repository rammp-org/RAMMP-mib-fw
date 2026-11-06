import sys,re,shutil,os,subprocess
t=sys.argv[1]; os.chdir(t)
idf=os.path.expanduser('~/esp/esp-idf')
if not os.path.exists('components/lwip'): shutil.copytree(idf+'/components/lwip','components/lwip',symlinks=True)
p='components/lwip/apps/dhcpserver/dhcpserver.c'; s=open(p).read()
s=s.replace('#define DHCPS_SERVER_PORT  67','#define DHCPS_SERVER_PORT  1067 /* TEST BUILD */').replace('#define DHCPS_CLIENT_PORT  68','#define DHCPS_CLIENT_PORT  1068 /* TEST BUILD */')
open(p,'w').write(s)
p='components/MIBConfig/include/MIBconfig.hpp'; s=open(p).read()
s=s.replace('ethernet_ip[4] = {192, 168, 4, 1};','ethernet_ip[4] = {192, 168, 1, 250};').replace('ethernet_gateway[4] = {192, 168, 4, 1};','ethernet_gateway[4] = {192, 168, 1, 250};')
open(p,'w').write(s)
p='main/main.cpp'; s=open(p).read()
s='''#include <cstdio>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
// TEST BUILD: heap and the event task's stack margin on the console.
static void netdiag(void *) {
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    TaskHandle_t evt = xTaskGetHandle("sys_evt");
    printf("NETDIAG up=%llu int_free=%u int_largest=%u int_min=%u evt_hwm=%u\\n", esp_timer_get_time() / 1000000ULL,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           evt ? (unsigned)uxTaskGetStackHighWaterMark(evt) : 0);
  }
}
'''+s
s=s.replace('  system.start();','  xTaskCreate(netdiag, "netdiag", 4096, nullptr, 2, nullptr);\n  system.start();')
open(p,'w').write(s)
