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

#include <stdbool.h>
#include <string.h>

#if __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "ssd1306_gl.h"

static float pixel_size = 1.0;
static uint8_t luma_pixmap[SSD1306_VIRT_COLUMNS*SSD1306_VIRT_PAGES*8];

void update_lumamap(ssd1306_t *ssd1306, const uint8_t luma_decay, const uint8_t luma_inc)
{
	uint8_t *column_ptr = luma_pixmap;
	for (int p = 0; p < SSD1306_VIRT_PAGES; p++) {
		for (int c = 0; c < SSD1306_VIRT_COLUMNS; c++) {
			uint8_t px_col = ssd1306->vram[p][c];
			for (int px_idx = 0; px_idx < 8*SSD1306_VIRT_COLUMNS; px_idx += SSD1306_VIRT_COLUMNS) {
				int16_t luma = column_ptr[px_idx];
				luma -= luma_decay;
				if (px_col & 0x1) {
					luma += luma_inc;
				}
				/* clamp value to [0, 255] */
				if (luma < 0) {
					luma = 0;
				} else if (luma > 255) {
					luma = 255;
				}
				column_ptr[px_idx] = luma;
				px_col >>= 1;
			}
			column_ptr++;
		}
		column_ptr += SSD1306_VIRT_COLUMNS*7;
	}
}

static inline void gl_set_bg_colour_(uint8_t invert, float opacity)
{
	if (invert) {
		glColor4f(1.0f, 1.0f, 1.0f, opacity);
	} else {
		glColor4f(0.0f, 0.0f, 0.0f, 1.0f);
	}
}

static inline void gl_set_fg_colour_(uint8_t invert, float opacity)
{
	if (invert) {
		glColor4f(0.0f, 0.0f, 0.0f, opacity);
	} else {
		glColor4f(1.0f, 1.0f, 1.0f, opacity);
	}
}

static float contrast_to_opacity_(uint8_t contrast)
{
	// Typically the screen will be clearly visible even at 0 contrast
	return contrast / 512.0 + 0.5;
}

void ssd1306_gl_render(ssd1306_t *ssd1306)
{
	if (!ssd1306_get_flag(ssd1306, SSD1306_FLAG_DISPLAY_ON)) {
		return;
	}

	glEnable (GL_BLEND);
	glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	
	float opacity = contrast_to_opacity_(ssd1306->contrast_register);
	int invert = ssd1306_get_flag (ssd1306, SSD1306_FLAG_DISPLAY_INVERTED);
	gl_set_bg_colour_(invert, opacity);

	glTranslatef (0, 0, 0);

	glBegin (GL_QUADS);
	glVertex2f (0, ssd1306->rows*pixel_size);
	glVertex2f (0, 0);
	glVertex2f (ssd1306->columns*pixel_size, 0);
	glVertex2f (ssd1306->columns*pixel_size, ssd1306->rows*pixel_size);
	
	uint8_t *px_ptr = luma_pixmap;
	float v_ofs = 0;
	while (v_ofs < ssd1306->rows*pixel_size) {
		float h_ofs = 0;
		while (h_ofs < ssd1306->columns*pixel_size) {
			gl_set_fg_colour_(invert, ((float)(*px_ptr))/255.0 * opacity);
			glVertex2f(h_ofs + pixel_size, v_ofs + pixel_size);
			glVertex2f(h_ofs, v_ofs + pixel_size);
			glVertex2f(h_ofs, v_ofs);
			glVertex2f(h_ofs + pixel_size, v_ofs);

			h_ofs += pixel_size;
			px_ptr++;
		}
		v_ofs += pixel_size;
	}
	glEnd ();
}

void ssd1306_gl_init(float pix_size)
{
	pixel_size = pix_size;
	memset(luma_pixmap, 0, sizeof(*luma_pixmap));
}
