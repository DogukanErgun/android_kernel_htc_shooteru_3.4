/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *     * Neither the name of Code Aurora Forum, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
 * OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
 * IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef MIPI_SHOOTERU_BLUE_H
#define MIPI_SHOOTERU_BLUE_H

int mipi_shooteru_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel);

#define DEFAULT_BRIGHTNESS              83

#define BRI_SETTING_MIN                 20
#define BRI_SETTING_DEF                 113
#define BRI_SETTING_MAX                 255

#define SHARP_PWM_MIN                   9	/* 3.5% of max pwm */
#define SHARP_PWM_DEFAULT               69	/* 27% of max pwm  */
#define SHARP_PWM_MAX                   194	/* 76% of max pwm */

#define AUO_PWM_MIN                     9	/* 3.5% of max pwm */
#define AUO_PWM_DEFAULT                 87	/* 34% of max pwm  */
#define AUO_PWM_MAX                     255	/* 100% of max pwm  */


#define PWM_MIN				8
#define PWM_DEFAULT			91
#define PWM_MAX				232

#endif  /* MIPI_SHOOTERU_BLUE_H */

