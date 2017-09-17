/*
	Copyright 2017 Delio Brignoli <brignoli.delio@gmail.com>

	Arduboy board implementation using simavr.

	This program is free software: you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	This program is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <libgen.h>
#include <time.h>

#include "sim_avr.h"
#include "avr_ioport.h"
#include "sim_hex.h"
#include "sim_gdb.h"
#include "sim_time.h"

#if __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ssd1306_virt.h"
#include "ssd1306_gl.h"

enum {
	BTN_UP = 0,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_A,
	BTN_B,
	BTN_COUNT,
};

#define MHZ_16 (16000000)
#define SSD1306_FRAME_PERIOD_US (7572)
#define GL_FRAME_PERIOD_US (SSD1306_FRAME_PERIOD_US*12)
#if 1
#define LUMA_INC (256*2/3)
#define LUMA_DECAY (256/3)
#else
#define LUMA_INC (255)
#define LUMA_DECAY (255)
#endif

struct button_info {
	avr_irq_t *irq;
	const char *name;
	char port_name;
	int port_idx;
};

static struct button_info buttons[BTN_COUNT] = {
	{NULL, "btn.up", 'F', 7}, /* BTN_UP */
	{NULL, "btn.down", 'F', 4},
	{NULL, "btn.left", 'F', 5},
	{NULL, "btn.right", 'F', 6},
	{NULL, "btn.a", 'E', 6},
	{NULL, "btn.b", 'B', 4},
};

int window_identifier;

avr_t * avr = NULL;
uint64_t start_time_ns;

ssd1306_t ssd1306;
bool yield_to_glut = false;
int win_width, win_height;
float pixel_size;

static avr_cycle_count_t update_luma(
		avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	update_lumamap(&ssd1306, LUMA_DECAY, LUMA_INC);
	return avr->cycle + avr_usec_to_cycles(avr, SSD1306_FRAME_PERIOD_US);
}

static avr_cycle_count_t schedule_render(
		avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	glutPostRedisplay();
	yield_to_glut = true;
	return avr->cycle + avr_usec_to_cycles(avr, GL_FRAME_PERIOD_US);
}

/*
Simavr's default sleep callback results in simulated time and
wall clock time to diverge over time. This replacement tries to
keep them in sync by sleeping for the time required to match the
expected sleep deadline in wall clock time.
*/
static void avr_callback_sleep_sync(
		avr_t * avr,
		avr_cycle_count_t howLong)
{
	struct timespec tp;

	/* figure out how log we should wait to match the sleep deadline */
	uint64_t deadline_ns = avr_cycles_to_nsec(avr, avr->cycle + howLong);
	clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	uint64_t runtime_ns = (tp.tv_sec*1000000000+tp.tv_nsec) - start_time_ns;
	if (runtime_ns >= deadline_ns) {
		return;
	}

	uint64_t sleep_us = (deadline_ns - runtime_ns)/1000;
	usleep(sleep_us);
	return;
}

static void glut_avr_run(void)
{
	yield_to_glut = false;
	while (!yield_to_glut) {
		avr->run(avr);
	}
}

void special_key(int key, bool release)
{
	avr_irq_t *irq = NULL;
	switch (key)
	{
		case GLUT_KEY_UP:
			irq = buttons[BTN_UP].irq;
			break;
		case GLUT_KEY_DOWN:
			irq = buttons[BTN_DOWN].irq;
			break;
		case GLUT_KEY_LEFT:
			irq = buttons[BTN_LEFT].irq;
			break;
		case GLUT_KEY_RIGHT:
			irq = buttons[BTN_RIGHT].irq;
			break;
	}
	if (irq) {
		avr_raise_irq(irq, !!release);
	}
}

void special_key_press(int key, int x, int y)
{
	special_key(key, false);
}

void special_key_release(int key, int x, int y)
{
	special_key(key, true);
}

/* Called on a key press */
void key_event(unsigned char key, bool release)
{
	avr_irq_t *irq = NULL;
	switch (key)
	{
		case 'q':
			exit (0);
			break;
		case 'z':
			irq = buttons[BTN_A].irq;
			break;
		case 'x':
			irq = buttons[BTN_B].irq;
			break;
	}
	if (irq) {
		avr_raise_irq(irq, !!release);
	}
}

void key_press(unsigned char key, int x, int y)
{
	key_event(key, false);
}

void key_release(unsigned char key, int x, int y)
{
	key_event(key, true);
}

/* Function called whenever redisplay needed */
void
displayCB (void)
{
	const uint8_t seg_remap_default = ssd1306_get_flag (
			&ssd1306, SSD1306_FLAG_SEGMENT_REMAP_0);
	const uint8_t seg_comscan_default = ssd1306_get_flag (
			&ssd1306, SSD1306_FLAG_COM_SCAN_NORMAL);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set up projection matrix
	glMatrixMode(GL_PROJECTION);
	// Start with an identity matrix
	glLoadIdentity();
	glOrtho(0, win_width, 0, win_height, 0, 10);
	// Apply vertical and horizontal display mirroring
	glScalef(seg_remap_default ? -1 : 1, seg_comscan_default ? 1 : -1, 1);
	glTranslatef(seg_remap_default ? -win_width : 0, seg_comscan_default ? 0: -win_height, 0);

	// Select modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	// Start with an identity matrix
	glLoadIdentity();
	ssd1306_gl_render(&ssd1306);
	glPopMatrix();
	glutSwapBuffers();
}

int
initGL (int w, int h, float pix_size)
{
	win_width = w * pix_size;
	win_height = h * pix_size;
	pixel_size = pix_size;

	// Double buffered, RGB disp mode.
	glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize (win_width, win_height);
	window_identifier = glutCreateWindow ("SSD1306 128x64 OLED");

	// Set window's display callback
	glutDisplayFunc (displayCB);
	// Set window's key callback
	glutKeyboardFunc(key_press);
	glutKeyboardUpFunc(key_release);
	glutSpecialFunc(special_key_press);
	glutSpecialUpFunc(special_key_release);
	glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF);
	glutIgnoreKeyRepeat(1);
	// Set Idle callback
	glutIdleFunc(glut_avr_run);

	ssd1306_gl_init (pix_size);

	return 1;
}

int
main (int argc, char *argv[])
{
	char boot_path[1024] = "ATmegaBOOT_168_atmega328.ihex";
	uint32_t boot_base, boot_size;

	int debug = 0;
	int verbose = 0;

	for (int i = 1; i < argc; i++) {
		if (!strcmp(argv[i] + strlen(argv[i]) - 4, ".hex"))
			strncpy(boot_path, argv[i], sizeof(boot_path));
		else if (!strcmp(argv[i], "-d"))
			debug++;
		else if (!strcmp(argv[i], "-v"))
			verbose++;
		else {
			fprintf(stderr, "%s: invalid argument %s\n", argv[0], argv[i]);
			exit(1);
		}
	}

	avr = avr_make_mcu_by_name ("atmega32u4");
	if (!avr)
	{
		exit (1);
	}

	uint8_t * boot = read_ihex_file(boot_path, &boot_size, &boot_base);
	if (!boot) {
		fprintf(stderr, "%s: Unable to load %s\n", argv[0], boot_path);
		exit(1);
	}
	printf("hex image 0x%05x: %d bytes\n", boot_base, boot_size);

	avr_init (avr);

	memcpy(avr->flash + boot_base, boot, boot_size);
	free(boot);
	avr->pc = boot_base;
	/* end of flash, remember we are writing /code/ */
	avr->codeend = avr->flashend;
	avr->log = 1 + verbose;
	avr->frequency = MHZ_16;
	avr->sleep = avr_callback_sleep_sync;
	avr->run_cycle_limit = avr_usec_to_cycles(avr, 2*GL_FRAME_PERIOD_US);

	ssd1306_init (avr, &ssd1306, 128, 64);

	// SSD1306 wired to the SPI bus, with the following additional pins:
	ssd1306_wiring_t wiring =
	{
		.chip_select.port = 'D',
		.chip_select.pin = 6,
		.data_instruction.port = 'D',
		.data_instruction.pin = 4,
		.reset.port = 'D',
		.reset.pin = 7,
	};

	for (int btn_idx=0; btn_idx<BTN_COUNT; btn_idx++) {
		struct button_info *binfo = &buttons[btn_idx];
		binfo->irq = avr_alloc_irq(&avr->irq_pool, 0, 1, &binfo->name);
		uint32_t iop_ctl = AVR_IOCTL_IOPORT_GETIRQ(binfo->port_name);
		avr_irq_t *iop_irq = avr_io_getirq(avr, iop_ctl, binfo->port_idx);
		avr_connect_irq(binfo->irq, iop_irq);
		/* pull up pin */
		avr_raise_irq(binfo->irq, 1);
	}

	ssd1306_connect (&ssd1306, &wiring);
	avr_cycle_timer_register_usec(avr, SSD1306_FRAME_PERIOD_US, update_luma, &ssd1306);
	avr_cycle_timer_register_usec(avr, GL_FRAME_PERIOD_US, schedule_render, &ssd1306);
	printf ("SSD1306 display demo\n   Press 'q' to quit\n");

	// Initialize GLUT system
	glutInit (&argc, argv);
	initGL (ssd1306.columns, ssd1306.rows, 2.0);
	// Take start time
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	start_time_ns = tp.tv_sec*1000000000+tp.tv_nsec;

	glutMainLoop();
}
