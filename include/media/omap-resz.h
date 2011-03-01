/*
 * omap_resz.h
 *
 * Copyright (c) 2006 Archos
 * Author: Matthias Welwarsky
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  THIS  SOFTWARE  IS PROVIDED   ``AS  IS'' AND   ANY  EXPRESS OR IMPLIED
 *  WARRANTIES,   INCLUDING, BUT NOT  LIMITED  TO, THE IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN
 *  NO  EVENT  SHALL   THE AUTHOR  BE    LIABLE FOR ANY   DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 *  NOT LIMITED   TO, PROCUREMENT OF  SUBSTITUTE GOODS  OR SERVICES; LOSS OF
 *  USE, DATA,  OR PROFITS; OR  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 *  ANY THEORY OF LIABILITY, WHETHER IN  CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *  You should have received a copy of the  GNU General Public License along
 *  with this program; if not, write  to the Free Software Foundation, Inc.,
 *  675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#ifndef _MEDIA_OMAP_RESZ_H
#define _MEDIA_OMAP_RESZ_H

#define VPSSRSZ_CMD_PARAMBLT_ASYNC	3
#define VPSSRSZ_CMD_WAIT_SYNC		4

typedef struct {
	unsigned long	addr;
	unsigned int	linestep;
	unsigned int	width, height;
} vpssrsz_rect_t;

typedef struct {
	short coeff[32];
} vpssrsz_coeff_t;

typedef struct {
	int size;
	vpssrsz_rect_t src_rect;
	vpssrsz_rect_t dst_rect;
	vpssrsz_coeff_t h_coeff;
	vpssrsz_coeff_t v_coeff;
	
	int	vbl_sync;
	int	format;
} vpssrsz_rsz_cmd_t;

typedef struct {
	int size;
	vpssrsz_rect_t src_rect;
	vpssrsz_rect_t dst_rect;
	unsigned int  h_rsz, v_rsz;
	
	vpssrsz_coeff_t h_coeff;
	vpssrsz_coeff_t v_coeff;
	
	int	vbl_sync;
	int 	format;
} vpssrsz_param_cmd_t;

#endif
