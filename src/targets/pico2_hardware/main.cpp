#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/timer.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
// Tu núcleo de navegación real e intacto
#include "core/navigation_cortex.hpp"
#include "core/sil_protocol.hpp" // Reutilizamos las estructuras binarias limpias
// Configuración de la red local en Comarruga
#define WIFI_SSID "TU_WIFI_DE_CASA"
#define WIFI_PASSWORD "TU_CONTRASEÑA"
#define HOST_IP "111.111.111.111" // La IP de tu PC de desarrollo
#define UDP_PORT 5005
// Variables globales para el control del tiempo determinista
volatile bool tick_ready = false;
// Alarma por hardware que salta estrictamente cada 10 ms (100 Hz)
bool repeating_timer_callback(struct repeating_timer *t) {
    tick_ready = true;
    return true; // Continuar el temporizador
}
int main() {
    // 1. Inicializar periféricos base y salida de consola por USB
    stdio_init_all();
    
    // 2. Despertar el chip de radio Infineon (Wi-Fi)
    if (cyw43_arch_init()) {
        printf("Error: Fallo al inicializar el chip Wi-Fi\n");
        return -1;
    }
    
    cyw43_arch_enable_sta_mode();
    printf("Conectando al Wi-Fi %s...\n", WIFI_SSID);
    
    if (cyw43_arch_wifi_connect_blocking(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Error: No se pudo conectar a la red inalámbrica\n");
        return -1;
    }
    printf("¡Conectado con éxito! IP Local: %s\n", ip4addr_ntoa(netif_ip4_addr(&cyw43_state.netif[0])));
    // 3. Inicializar aquí tus drivers físicos (SPI para IMU, UART para GPS)
    // [Próxima fase: spi_init, uart_init]
    // 4. Configurar el bucle temporal estricto de 100 Hz
    struct repeating_timer timer;
    add_repeating_timer_ms(-10, repeating_timer_callback, NULL, &timer);
    printf("Iniciando NavigationCortex en hardware real...\n");
    // Bucle infinito de ejecución ciberfísica
    while (true) {
        // Espera de bajo consumo hasta que salte el temporizador
        if (tick_ready) {
            tick_ready = false;
            // A. Leer datos reales de los sensores por los buses físicos
            // SilSensorPacket sensor_data = read_hardware_sensors();
            // B. Ejecutar el tick crítico de tu Cortex de siempre
            // navigation_cortex_tick(&sensor_data);
            // C. Si el Cortex genera telemetría, enviarla por el aire
            // cyw43_arch_lwip_begin();
            // udp_send_packet_via_wifi(&tu_paquete_32B);
            // cyw43_arch_lwip_end();
        }
        
        // Mantener la pila de red Wi-Fi viva en los tiempos muertos
        cyw43_arch_poll();
    }
    return 0;
}
