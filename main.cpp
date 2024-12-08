#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "bsp/board_api.h"
#include "hardware/gpio.h"
#include "rpico-servo/servo.h"
#include "tusb.h"
uint8_t packet[4];
Servo servo(12);

enum {
    BLINK_NOT_MOUNTED = 250,
    BLINK_MOUNTED = 1000,
    BLINK_SUSPENDED = 2500,
};

enum {
    C = 0,
    C_SHARP = 1,
    D = 2,
    D_SHARP = 3,
    E = 4,
    F = 5,
    F_SHARP = 6,
    G = 7,
    G_SHARP = 8,
    A = 9,
    A_SHARP = 10,
    B = 11
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;

void led_blinking_task(void);
void midi_task(void);
void solenoid_task(void);

/*------------- MAIN -------------*/
int main(void) {
    board_init();
    gpio_init(9);
    gpio_set_dir(9, GPIO_OUT);
    servo.init();
    // init device stack on configured roothub port
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUD_RHPORT, &dev_init);
    printf("START\n");
    servo.write(90);

    if (board_init_after_tusb) {
        board_init_after_tusb();
    }

    while (1) {
        tud_task();  // tinyusb device task
        led_blinking_task();
        midi_task();
        solenoid_task();
    }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void) {
    blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void) {
    blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en) {
    (void)remote_wakeup_en;
    blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void) {
    blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// MIDI Task
//--------------------------------------------------------------------+

void midi_task(void) {
    // The MIDI interface always creates input and output port/jack descriptors
    // regardless of these being used or not. Therefore incoming traffic should be read
    // (possibly just discarded) to avoid the sender blocking in IO
    while (tud_midi_available()) {
        uint8_t received_packet[4];
        tud_midi_packet_read(received_packet);
        memcpy(packet, received_packet, 4);
        // Print received MIDI packet
        printf("Received MIDI packet: %02X %02X %02X %02X\n", packet[0], packet[1], packet[2], packet[3]);
        printf("Note: %d\n", packet[2] % 12);
    }
}

void solenoid_task(void) {
    static uint32_t start_ms = 0;
    static bool note_on = false;

    if ((packet[1] & 0xF0) == 0x80 || (((packet[1] & 0xF0) == 0x90) && packet[3] == 0)) {
        // note off
        note_on = false;
        gpio_put(9, 0);
    } else if ((packet[1] & 0xF0) == 0x90) {
        // note on
        if (packet[2] % 12 == E) {
            static uint32_t note_on_time = 0;
            if (note_on_time == 0 && !note_on) {
                gpio_put(9, 1);
                note_on = true;
                printf("waiting\n");
                note_on_time = board_millis();
            } else if (board_millis() - note_on_time >= 20) {
                gpio_put(9, 0);
                printf("time over\n");
                note_on_time = 0;
            }
        } else if (packet[2] % 12 == G) {
            servo.write(60);
        } else if (packet[2] % 12 == A) {
            servo.write(90);
        } else if (packet[2] % 12 == B) {
            servo.write(120);
        } else if (packet[2] % 12 == F_SHARP) {
            servo.write(60);
            gpio_put(9, 1);
            note_on = true;
        } else if (packet[2] % 12 == G_SHARP) {
            servo.write(90);
            gpio_put(9, 1);
            note_on = true;
        } else if (packet[2] % 12 == A_SHARP) {
            servo.write(120);
            gpio_put(9, 1);
            note_on = true;
        } else {
            gpio_put(9, 1);
            note_on = true;
        }
    }
}
//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
void led_blinking_task(void) {
    static uint32_t start_ms = 0;
    static bool led_state = false;

    // Blink every interval ms
    if (board_millis() - start_ms < blink_interval_ms)
        return;  // not enough time
    start_ms += blink_interval_ms;

    board_led_write(led_state);
    //   gpio_put(9, led_state);
    led_state = 1 - led_state;  // toggle
}
