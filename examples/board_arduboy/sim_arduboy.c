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
#include <unistd.h>
#include <string.h>
#include <time.h>
#include <sys/socket.h>
#include <fcntl.h>

#include "sim_avr.h"
#include "avr_ioport.h"
#include "avr_extint.h"
#include "sim_hex.h"
#include "sim_gdb.h"
#include "sim_time.h"

#if __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <pthread.h>

#include "ssd1306_virt.h"
#include "ssd1306_gl.h"

#define USE_THREADS 0

#define MHZ_16 (16000000)
#define SSD1306_FRAME_PERIOD_US (7572)
#define GL_FRAME_PERIOD_US (SSD1306_FRAME_PERIOD_US*12)

#define LUMA_INC (256*2/3)
#define LUMA_DECAY (256/3)

enum button_e {
	BTN_UP = 0,
	BTN_DOWN,
	BTN_LEFT,
	BTN_RIGHT,
	BTN_A,
	BTN_B,
	BTN_COUNT,
};

struct button_info {
	enum button_e btn_id;
	avr_irq_t *irq;
	const char *name;
	char port_name;
	int port_idx;
	bool pressed;
};

static struct button_info buttons[BTN_COUNT] = {
	{0, NULL, "btn.up", 'F', 7}, /* BTN_UP */
	{1, NULL, "btn.down", 'F', 4},
	{2, NULL, "btn.left", 'F', 5},
	{3, NULL, "btn.right", 'F', 6},
	{4, NULL, "btn.a", 'E', 6},
	{5, NULL, "btn.b", 'B', 4},
};

struct btn_event {
	enum button_e btn_id;
	avr_irq_t *irq;
	bool pressed;
};

struct app_state {
	avr_t *avr;
#if USE_THREADS
	int socketpair[2];
	pthread_t avr_thread;
#endif
	uint64_t start_time_ns;
	int window_identifier;
	int win_width, win_height;
	ssd1306_t ssd1306;
	float pixel_size;
	bool yield;
};

static struct app_state app_s;

static avr_cycle_count_t update_luma(
		avr_t *avr,
		avr_cycle_count_t when,
		void *param)
{
	update_lumamap(&app_s.ssd1306, LUMA_DECAY, LUMA_INC);
	return avr->cycle + avr_usec_to_cycles(avr, SSD1306_FRAME_PERIOD_US);
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
	uint64_t runtime_ns = (tp.tv_sec*1000000000+tp.tv_nsec) - app_s.start_time_ns;
	if (runtime_ns >= deadline_ns) {
		return;
	}

	uint64_t sleep_us = (deadline_ns - runtime_ns)/1000;
	usleep(sleep_us);
	return;
}

static void avr_run_loop(void)
{
	avr_t *avr = app_s.avr;
	app_s.yield = false;
	while (!app_s.yield) {
		avr->run(avr);
		int state = avr->state;
		if (state == cpu_Done || state == cpu_Crashed)
			break;

#if USE_THREADS
		ssize_t r_sz;
		do {
			struct btn_event e;
			r_sz = recv(app_s.socketpair[1], &e, sizeof(e), 0);
			if (r_sz == sizeof(e)) {
				avr_raise_irq(e.irq, !e.pressed);
			}
		} while (r_sz > 0);
#endif
	}
}

#if USE_THREADS
static void *avr_run_thread(void *app)
{
	struct app_state *s = app;
	// Take start time
	struct timespec tp;
	clock_gettime(CLOCK_MONOTONIC_RAW, &tp);
	s->start_time_ns = tp.tv_sec*1000000000+tp.tv_nsec;

	avr_run_loop();
	return NULL;
}
#endif

static inline struct button_info *special_key_to_button_info(int key)
{
	switch (key)
	{
		case GLUT_KEY_UP:
			return &buttons[BTN_UP];
		case GLUT_KEY_DOWN:
			return &buttons[BTN_DOWN];
		case GLUT_KEY_LEFT:
			return &buttons[BTN_LEFT];
		case GLUT_KEY_RIGHT:
			return &buttons[BTN_RIGHT];
	}
	return NULL;
}

static inline struct button_info *key_to_button_info(unsigned char key)
{
	switch (key)
	{
		case 'q':
			exit(0);
			return NULL;
		case 'z':
			return &buttons[BTN_A];
		case 'x':
			return &buttons[BTN_B];
	}
	return NULL;
}

void notify_button_event(struct button_info *btn, bool pressed)
{
	if (btn && btn->pressed != pressed) {
		struct btn_event e = {btn->btn_id, btn->irq, pressed};
#if USE_THREADS
		send(app_s.socketpair[0], &e, sizeof(e), 0);
#else
		avr_raise_irq(e.irq, !e.pressed);
#endif
		btn->pressed = pressed;
	}
}

void special_key_press(int key, int x, int y)
{
	struct button_info *btn = special_key_to_button_info(key);
	notify_button_event(btn, true);
}

void special_key_release(int key, int x, int y)
{
	struct button_info *btn = special_key_to_button_info(key);
	notify_button_event(btn, false);
}

void key_press(unsigned char key, int x, int y)
{
	struct button_info *btn = key_to_button_info(key);
	notify_button_event(btn, true);
}

void key_release(unsigned char key, int x, int y)
{
	struct button_info *btn = key_to_button_info(key);
	notify_button_event(btn, false);
}

/* Function called whenever redisplay needed */
void
displayCB (void)
{
	const uint8_t seg_remap_default = ssd1306_get_flag (
			&app_s.ssd1306, SSD1306_FLAG_SEGMENT_REMAP_0);
	const uint8_t seg_comscan_default = ssd1306_get_flag (
			&app_s.ssd1306, SSD1306_FLAG_COM_SCAN_NORMAL);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set up projection matrix
	glMatrixMode(GL_PROJECTION);
	// Start with an identity matrix
	glLoadIdentity();
	glOrtho(0, app_s.win_width, 0, app_s.win_height, 0, 10);
	// Apply vertical and horizontal display mirroring
	glScalef(seg_remap_default ? -1 : 1, seg_comscan_default ? 1 : -1, 1);
	glTranslatef(seg_remap_default ? -app_s.win_width : 0, seg_comscan_default ? 0: -app_s.win_height, 0);

	// Select modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	// Start with an identity matrix
	glLoadIdentity();
	ssd1306_gl_render(&app_s.ssd1306);
	glPopMatrix();
	glutSwapBuffers();
}

#if USE_THREADS

void timerCB(int i)
{
	// restart timer
	glutTimerFunc(1000/GL_FRAME_PERIOD_US, timerCB, 0);
	glutPostRedisplay();
}

#else

static avr_cycle_count_t schedule_render(
			avr_t *avr,
			avr_cycle_count_t when,
			void *param)
{
	glutPostRedisplay();
	app_s.yield = true;
	return avr->cycle + avr_usec_to_cycles(avr, GL_FRAME_PERIOD_US);
}

#endif

int
initGL (int w, int h, float pix_size)
{
	app_s.win_width = w * pix_size;
	app_s.win_height = h * pix_size;
	app_s.pixel_size = pix_size;

	// Double buffered, RGB disp mode.
	glutInitDisplayMode (GLUT_RGB | GLUT_DOUBLE);
	glutInitWindowSize (app_s.win_width, app_s.win_height);
	app_s.window_identifier = glutCreateWindow ("Sim-Arduboy");

	// Set window's display callback
	glutDisplayFunc (displayCB);
	// Set window's key callback
	glutKeyboardFunc(key_press);
	glutKeyboardUpFunc(key_release);
	glutSpecialFunc(special_key_press);
	glutSpecialUpFunc(special_key_release);
	//glutSetKeyRepeat(GLUT_KEY_REPEAT_OFF);
	glutIgnoreKeyRepeat(1);

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
	int gdb_port = 1234;


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

	avr_t *avr = avr_make_mcu_by_name ("atmega32u4");
	if (!avr) {
		exit (1);
	}
	app_s.avr = avr;

	uint8_t * boot = read_ihex_file(boot_path, &boot_size, &boot_base);
	if (!boot) {
		fprintf(stderr, "%s: Unable to load %s\n", argv[0], boot_path);
		exit(1);
	}
	printf("hex image 0x%05x: %d bytes\n", boot_base, boot_size);

	avr_init (avr);

	/*
	BTN_A is wired to INT6 which apparently defaults to level triggered.
	This means that while button A is pressed the interrupt triggers
	continuously. This is very expensive to simulate so we set non-strict
	level trigger mode for INT6.

	Why doesn't this affect real h/w?
	*/
	avr_extint_set_strict_lvl_trig(avr, EXTINT_IRQ_OUT_INT6, 0);

	memcpy(avr->flash + boot_base, boot, boot_size);
	free(boot);
	avr->pc = boot_base;
	/* end of flash, remember we are writing /code/ */
	avr->codeend = avr->flashend;
	avr->log = 1 + verbose;
	avr->frequency = MHZ_16;
	avr->sleep = avr_callback_sleep_sync;
	avr->run_cycle_limit = avr_usec_to_cycles(avr, 2*GL_FRAME_PERIOD_US);

	ssd1306_init (avr, &app_s.ssd1306, 128, 64);

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

	ssd1306_connect (&app_s.ssd1306, &wiring);
	avr_cycle_timer_register_usec(avr, SSD1306_FRAME_PERIOD_US, update_luma, &app_s.ssd1306);
	printf ("SSD1306 display demo\n   Press 'q' to quit\n");

	// Initialize GLUT system
	glutInit (&argc, argv);
	initGL (app_s.ssd1306.columns, app_s.ssd1306.rows, 2.0);

	avr->gdb_port = gdb_port;
	if (debug) {
		avr->state = cpu_Stopped;
		avr_gdb_init(avr);
	}

#if USE_THREADS
	glutTimerFunc(1000/GL_FRAME_PERIOD_US, timerCB, 0);
	socketpair(AF_UNIX, SOCK_DGRAM, 0, app_s.socketpair);
	fcntl(app_s.socketpair[1], F_SETFL, O_NONBLOCK);
	pthread_create(&app_s.avr_thread, NULL, avr_run_thread, &app_s);
#else
	glutIdleFunc(avr_run_loop);
	avr_cycle_timer_register_usec(avr, GL_FRAME_PERIOD_US, schedule_render, &app_s.ssd1306);
#endif

	glutMainLoop();
}
