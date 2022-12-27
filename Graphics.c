#include <stdlib.h>
#include <stdlib.h>
#include <math.h>
#include "stm32f4xx.h"                  // Device header
#include "Graphics.h"
#include "lcd.h"
#include "tools.h"

static uint16_t* buf_ref;
static uint8_t iPitch = 64;
static mat4x4 matProj = {0};

static float fNear;
static float fFar;
static float fFov;
static float fAspectRatio;
static float fFovRad;

void graphics_init(void){
  		// Projection Matrix
		fNear = 0.1f;
		fFar = 100.0f;
		fFov = 120.0f;
		fAspectRatio = 1.0f;
		fFovRad = tanf(fFov * 0.5f / 180.0f * 3.14159f);
    
		matProj.m[0][0] = 1.0f/(fAspectRatio * fFovRad);
		matProj.m[1][1] = 1.0f/fFovRad;
		matProj.m[2][2] = (-fNear - fFar) /(fFar - fNear);
		matProj.m[3][2] = 2.0f * fFar * fNear / (fFar - fNear);
		matProj.m[2][3] = 1.0f;
		matProj.m[3][3] = 0.0f;
}

void set_buf_ref(uint16_t* buf_r)
{
  buf_ref = buf_r;
}

void MultiplyMatrixVector(vec3 *i, vec3 *o, mat4x4 *m)
{
  o->x = i->x * m->m[0][0] + i->y * m->m[1][0] + i->z * m->m[2][0] + m->m[3][0];
  o->y = i->x * m->m[0][1] + i->y * m->m[1][1] + i->z * m->m[2][1] + m->m[3][1];
  o->z = i->x * m->m[0][2] + i->y * m->m[1][2] + i->z * m->m[2][2] + m->m[3][2];
  float w = i->x * m->m[0][3] + i->y * m->m[1][3] + i->z * m->m[2][3] + m->m[3][3];

  if (w != 0.0f)
  {
    o->x /= w; o->y /= w; o->z /= w;
  }
}

void rotate(vec3* point, float x, float y, float z){
//  
//  float rad = 0;
//  
//  rad = x;
//  point->y = cos(rad) * point->y - sin(rad) * point->z;
//  point->z = sin(rad) * point->y + cos(rad) * point->z;
//  rad = y;
//  point->x = cos(rad) * point->x + sin(rad) * point->z;
//  point->z = -sin(rad) * point->x + cos(rad) * point->z;
//  rad = z;
//  point->x = cos(rad) * point->x - sin(rad) * point->y;
//  point->y = sin(rad) * point->x + cos(rad) * point->y;
    
  
    vec3 pRotatedZ = {0,0,0};
    vec3 pRotatedZX = {0,0,0};
    vec3 pRotatedZXY = {0,0,0};
    
    mat4x4 matRotZ = {0};
    mat4x4 matRotX = {0};
    mat4x4 matRotY = {0};
  		// Rotation Z
		matRotZ.m[0][0] = cosf(z);
		matRotZ.m[0][1] = sinf(z);
		matRotZ.m[1][0] = -sinf(z);
		matRotZ.m[1][1] = cosf(z);
		matRotZ.m[2][2] = 1.0f;
		matRotZ.m[3][3] = 1.0f;

		// Rotation X
		matRotX.m[0][0] = 1.0f;
		matRotX.m[1][1] = cosf(x);
		matRotX.m[1][2] = sinf(x);
		matRotX.m[2][1] = -sinf(x);
		matRotX.m[2][2] = cosf(x);
		matRotX.m[3][3] = 1.0f;

		// Rotation Y
		matRotY.m[0][0] = cosf(y);
		matRotY.m[0][2] = sinf(y);
		matRotY.m[1][1] = 1.0f;
		matRotY.m[2][0] = -sinf(y);
		matRotY.m[2][2] = cosf(y);
		matRotY.m[3][3] = 1.0f;
    
    // Rotate in Z-Axis
    MultiplyMatrixVector(point, &pRotatedZ, &matRotZ);
  	// Rotate in X-Axis
    MultiplyMatrixVector(&pRotatedZ, &pRotatedZX, &matRotX);
  	// Rotate in Y-Axis
    MultiplyMatrixVector(&pRotatedZX, &pRotatedZXY, &matRotY);	

    *point = pRotatedZXY;
    *point = *point;
}


//Project x and y co-ordinate of a 3D point onto a screen given the camera co-ordinates
vec3 project(vec3 point){
    // Project triangles from 3D --> 2D
    vec3 pTranslated = {0,0,0};
    vec3 pProjected = {0,0,0};
    
    pTranslated = point;
    pTranslated.z += 3.5f;
    MultiplyMatrixVector(&pTranslated, &pProjected, &matProj);
    return pProjected;
}

//void GFX_DrawLine(int x1, int y1, int x2, int y2, uint8_t ucColor)
//{
//  int temp;
//  int dx = x2 - x1;
//  int dy = y2 - y1;
//  int error;
//  uint8_t *p;
//  int xinc, yinc, shift;
//  int y, x;
//  
//  if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= SSD1327_LCDWIDTH || x2 >= SSD1327_LCDWIDTH || y1 >= SSD1327_LCDWIDTH || y2 >= SSD1327_LCDWIDTH)
//     return;

//  if(abs(dx) > abs(dy)) {
//    // X major case
//    if(x2 < x1) {
//      dx = -dx;
//      temp = x1;
//      x1 = x2;
//      x2 = temp;
//      temp = y1;
//      y1 = y2;
//      y2 = temp;
//    }

//    y = y1;
//    dy = (y2 - y1);
//    error = dx >> 1;
//    yinc = 1;
//    if (dy < 0)
//    {
//      dy = -dy;
//      yinc = -1;
//    }
//    p = &buf_ref[(x1/2) + (y1 * iPitch)]; // point to current spot in back buffer
//    shift = (x1 & 1) ? 0:4; // current bit offset
//    for(x=x1; x1 <= x2; x1++) {
//      *p &= (0xf0 >> shift);
//      *p |= (ucColor << shift);
//      shift = 4-shift;
//      if (shift == 4) // time to increment pointer
//         p++;
//      error -= dy;
//      if (error < 0)
//      {
//        error += dx;
//        if (yinc > 0)
//           p += iPitch;
//        else
//           p -= iPitch;
//        y += yinc;
//      }
//    } // for x1    
//  }
//  else {
//    // Y major case
//    if(y1 > y2) {
//      dy = -dy;
//      temp = x1;
//      x1 = x2;
//      x2 = temp;
//      temp = y1;
//      y1 = y2;
//      y2 = temp;
//    } 

//    p = &buf_ref[(x1/2) + (y1 * iPitch)]; // point to current spot in back buffer
//    shift = (x1 & 1) ? 0:4; // current bit offset
//    dx = (x2 - x1);
//    error = dy >> 1;
//    xinc = 1;
//    if (dx < 0)
//    {
//      dx = -dx;
//      xinc = -1;
//    }
//    for(x = x1; y1 <= y2; y1++) {
//      *p &= (0xf0 >> shift); // set the pixel
//      *p |= (ucColor << shift);
//      error -= dx;
//      p += iPitch; // y1++
//      if (error < 0)
//      {
//        error += dy;
//        x += xinc;
//        shift = 4-shift;
//        if (xinc == 1)
//        {
//          if (shift == 4) // time to increment pointer
//            p++;
//        }
//        else
//        {
//          if (shift == 0)
//            p--;
//        }
//      }
//    } // for y
//  } // y major case
//} /* GFX_DrawLine() */


//void GFX_DrawChar(int x, int y, char chr, uint8_t color, uint8_t background, uint8_t size)
//{
//  
//	if(chr > 0x7E) return; // chr > '~'

//	for(uint8_t i=0; i<font_8x5[1]; i++ )
//	{
//        uint8_t line = (uint8_t)font_8x5[(chr-0x20) * font_8x5[1] + i + 2];

//        for(int8_t j=0; j<font_8x5[0]; j++, line >>= 1)
//        {
//            srand(i+j+rand());
//            if(line & 1)
//            {
//            	if(size == 1)
//            		SSD1327_DrawPixel(x+i, y+j, color);
//            	else
//            		GFX_DrawFillRectangle(x+i*size, y+j*size, size, size, (rand()%16)*(rand()%2) );
//            }
//            else if(background > 0)
//            {
//            	if(size == 1)
//                SSD1327_DrawPixel(x+i, y+j, background);
//              else
//                GFX_DrawFillRectangle(x+i*size, y+j*size, size, size, background);
//            }
//        }
//    }
//}

//void GFX_DrawString(int x, int y, char* str, uint8_t color, uint8_t background, uint8_t size)
//{
//	int x_tmp = x;
//	char znak;
//	znak = *str;
//	while(*str++)
//	{
//		GFX_DrawChar(x_tmp, y, znak, color, background, size);
//		x_tmp += ((uint8_t)font_8x5[1] * size) + 1;
//		if(background == 0)
//		{
//			for(uint8_t i=0; i<(font_8x5[0]*size); i++)
//			{
//				SSD1327_DrawPixel(x_tmp-1, y+i, BLACK);
//			}
//		}
//		znak = *str;
//	}
//}

//void GFX_DrawFastVLine(int x_start, int y_start, int h, uint8_t color)
//{
//	GFX_DrawLine(x_start, y_start, x_start, y_start+h-1, color);
//}

//void GFX_DrawFastHLine(int x_start, int y_start, int w, uint8_t color)
//{
//	GFX_DrawLine(x_start, y_start, x_start+w-1, y_start, color);
//}

//void GFX_DrawRectangle(int x, int y, uint16_t w, uint16_t h, uint8_t color)
//{

//    GFX_DrawFastHLine(x, y, w, color);
//    GFX_DrawFastHLine(x, y+h-1, w, color);
//    GFX_DrawFastVLine(x, y, h, color);
//    GFX_DrawFastVLine(x+w-1, y, h, color);

//}

//void GFX_DrawFillRectangle(int x, int y, uint16_t w, uint16_t h, uint8_t color)
//{
//    for (int i=x; i<x+w; i++) {
//    	GFX_DrawFastVLine(i, y, h, color);
//    }

//}

//void GFX_DrawCircle(int x0, int y0, uint16_t r, uint8_t color)
//{
//    int16_t f = 1 - r;
//    int16_t ddF_x = 1;
//    int16_t ddF_y = -2 * r;
//    int16_t x = 0;
//    int16_t y = r;

//    SSD1327_DrawPixel(x0  , y0+r, color);
//    SSD1327_DrawPixel(x0  , y0-r, color);
//    SSD1327_DrawPixel(x0+r, y0  , color);
//    SSD1327_DrawPixel(x0-r, y0  , color);

//    while (x<y) {
//        if (f >= 0) {
//            y--;
//            ddF_y += 2;
//            f += ddF_y;
//        }
//        x++;
//        ddF_x += 2;
//        f += ddF_x;

//        SSD1327_DrawPixel(x0 + x, y0 + y, color);
//        SSD1327_DrawPixel(x0 - x, y0 + y, color);
//        SSD1327_DrawPixel(x0 + x, y0 - y, color);
//        SSD1327_DrawPixel(x0 - x, y0 - y, color);
//        SSD1327_DrawPixel(x0 + y, y0 + x, color);
//        SSD1327_DrawPixel(x0 - y, y0 + x, color);
//        SSD1327_DrawPixel(x0 + y, y0 - x, color);
//        SSD1327_DrawPixel(x0 - y, y0 - x, color);
//    }

//}


//void GFX_DrawCircleHelper( int x0, int y0, uint16_t r, uint8_t cornername, uint8_t color)
//{
//    int16_t f     = 1 - r;
//    int16_t ddF_x = 1;
//    int16_t ddF_y = -2 * r;
//    int16_t x     = 0;
//    int16_t y     = r;

//    while (x<y) {
//        if (f >= 0) {
//            y--;
//            ddF_y += 2;
//            f     += ddF_y;
//        }
//        x++;
//        ddF_x += 2;
//        f     += ddF_x;
//        if (cornername & 0x4) {
//            SSD1327_DrawPixel(x0 + x, y0 + y, color);
//            SSD1327_DrawPixel(x0 + y, y0 + x, color);
//        }
//        if (cornername & 0x2) {
//            SSD1327_DrawPixel(x0 + x, y0 - y, color);
//            SSD1327_DrawPixel(x0 + y, y0 - x, color);
//        }
//        if (cornername & 0x8) {
//            SSD1327_DrawPixel(x0 - y, y0 + x, color);
//            SSD1327_DrawPixel(x0 - x, y0 + y, color);
//        }
//        if (cornername & 0x1) {
//            SSD1327_DrawPixel(x0 - y, y0 - x, color);
//            SSD1327_DrawPixel(x0 - x, y0 - y, color);
//        }
//    }
//}


//void GFX_DrawFillCircleHelper(int x0, int y0, uint16_t r, uint8_t cornername, int16_t delta, uint8_t color)
//{

//    int16_t f     = 1 - r;
//    int16_t ddF_x = 1;
//    int16_t ddF_y = -2 * r;
//    int16_t x     = 0;
//    int16_t y     = r;

//    while (x<y) {
//        if (f >= 0) {
//            y--;
//            ddF_y += 2;
//            f     += ddF_y;
//        }
//        x++;
//        ddF_x += 2;
//        f     += ddF_x;

//        if (cornername & 0x1) {
//            GFX_DrawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
//            GFX_DrawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
//        }
//        if (cornername & 0x2) {
//            GFX_DrawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
//            GFX_DrawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
//        }
//    }
//}


//void GFX_DrawFillCircle(int x0, int y0, uint16_t r, uint8_t color)
//{

//	GFX_DrawFastVLine(x0, y0-r, 2*r+1, color);
//    GFX_DrawFillCircleHelper(x0, y0, r, 3, 0, color);
//}


//void GFX_DrawRoundRectangle(int x, int y, uint16_t w, uint16_t h, uint16_t r, uint8_t color)
//{
//	GFX_DrawFastHLine(x+r  , y    , w-2*r, color); // Top
//    GFX_DrawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
//    GFX_DrawFastVLine(x    , y+r  , h-2*r, color); // Left
//    GFX_DrawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
//    // draw four corners
//    GFX_DrawCircleHelper(x+r    , y+r    , r, 1, color);
//    GFX_DrawCircleHelper(x+w-r-1, y+r    , r, 2, color);
//    GFX_DrawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
//    GFX_DrawCircleHelper(x+r    , y+h-r-1, r, 8, color);
//}


//void GFX_DrawFillRoundRectangle(int x, int y, uint16_t w, uint16_t h, uint16_t r, uint8_t color)
//{
//    // smarter version

//	GFX_DrawFillRectangle(x+r, y, w-2*r, h, color);

//    // draw four corners
//	GFX_DrawFillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
//	GFX_DrawFillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
//}


//void GFX_DrawTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint8_t color)
//{
//	GFX_DrawLine(x0, y0, x1, y1, color);
//    GFX_DrawLine(x1, y1, x2, y2, color);
//    GFX_DrawLine(x2, y2, x0, y0, color);
//}

//void GFX_DrawFillTriangle(int x0, int y0, int x1, int y1, int x2, int y2, uint8_t color)
//{

//    int16_t a, b, y, last;

//    // Sort coordinates by Y order (y2 >= y1 >= y0)
//    if (y0 > y1) {
//    	_swap_int(y0, y1); _swap_int(x0, x1);
//    }
//    if (y1 > y2) {
//    	_swap_int(y2, y1); _swap_int(x2, x1);
//    }
//    if (y0 > y1) {
//    	_swap_int(y0, y1); _swap_int(x0, x1);
//    }

//    if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
//        a = b = x0;
//        if(x1 < a)      a = x1;
//        else if(x1 > b) b = x1;
//        if(x2 < a)      a = x2;
//        else if(x2 > b) b = x2;
//        GFX_DrawFastHLine(a, y0, b-a+1, color);
//        return;
//    }

//    int16_t
//    dx01 = x1 - x0,
//    dy01 = y1 - y0,
//    dx02 = x2 - x0,
//    dy02 = y2 - y0,
//    dx12 = x2 - x1,
//    dy12 = y2 - y1;
//    int32_t
//    sa   = 0,
//    sb   = 0;

//    // For upper part of triangle, find scanline crossings for segments
//    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
//    // is included here (and second loop will be skipped, avoiding a /0
//    // error there), otherwise scanline y1 is skipped here and handled
//    // in the second loop...which also avoids a /0 error here if y0=y1
//    // (flat-topped triangle).
//    if(y1 == y2) last = y1;   // Include y1 scanline
//    else         last = y1-1; // Skip it

//    for(y=y0; y<=last; y++) {
//        a   = x0 + sa / dy01;
//        b   = x0 + sb / dy02;
//        sa += dx01;
//        sb += dx02;
//        /* longhand:
//        a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
//        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
//        */
//        if(a > b) _swap_int(a,b);
//        GFX_DrawFastHLine(a, y, b-a+1, color);
//    }

//    // For lower part of triangle, find scanline crossings for segments
//    // 0-2 and 1-2.  This loop is skipped if y1=y2.
//    sa = dx12 * (y - y1);
//    sb = dx02 * (y - y0);
//    for(; y<=y2; y++) {
//        a   = x1 + sa / dy12;
//        b   = x0 + sb / dy02;
//        sa += dx12;
//        sb += dx02;
//        /* longhand:
//        a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
//        b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
//        */
//        if(a > b) _swap_int(a,b);
//        GFX_DrawFastHLine(a, y, b-a+1, color);
//    }
//}