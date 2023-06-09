#ifndef DSI_PHY_14NM_XML
#define DSI_PHY_14NM_XML

/* Autogenerated file, DO NOT EDIT manually!

This file was generated by the rules-ng-ng headergen tool in this git repository:
http://github.com/freedreno/envytools/
git clone https://github.com/freedreno/envytools.git

The rules-ng-ng source files this header was generated from are:
- /home/robclark/src/mesa/mesa/src/freedreno/registers/msm.xml                   (    944 bytes, from 2022-07-23 20:21:46)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/freedreno_copyright.xml   (   1572 bytes, from 2022-07-23 20:21:46)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/mdp/mdp4.xml              (  20912 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/mdp/mdp_common.xml        (   2849 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/mdp/mdp5.xml              (  37461 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi.xml               (  18746 bytes, from 2022-04-28 17:29:36)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_v2.xml        (   3236 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_28nm_8960.xml (   4935 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_28nm.xml      (   7004 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_20nm.xml      (   3712 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_14nm.xml      (   5381 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_10nm.xml      (   4499 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/dsi_phy_7nm.xml       (  11007 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/sfpb.xml              (    602 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/dsi/mmss_cc.xml           (   1686 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/hdmi/qfprom.xml           (    600 bytes, from 2022-03-08 17:40:42)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/hdmi/hdmi.xml             (  42350 bytes, from 2022-09-20 17:45:56)
- /home/robclark/src/mesa/mesa/src/freedreno/registers/edp/edp.xml               (  10416 bytes, from 2022-03-08 17:40:42)

Copyright (C) 2013-2022 by the following authors:
- Rob Clark <robdclark@gmail.com> (robclark)
- Ilia Mirkin <imirkin@alum.mit.edu> (imirkin)

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the
"Software"), to deal in the Software without restriction, including
without limitation the rights to use, copy, modify, merge, publish,
distribute, sublicense, and/or sell copies of the Software, and to
permit persons to whom the Software is furnished to do so, subject to
the following conditions:

The above copyright notice and this permission notice (including the
next paragraph) shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#define REG_DSI_14nm_PHY_CMN_REVISION_ID0			0x00000000

#define REG_DSI_14nm_PHY_CMN_REVISION_ID1			0x00000004

#define REG_DSI_14nm_PHY_CMN_REVISION_ID2			0x00000008

#define REG_DSI_14nm_PHY_CMN_REVISION_ID3			0x0000000c

#define REG_DSI_14nm_PHY_CMN_CLK_CFG0				0x00000010
#define DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_3_0__MASK		0x000000f0
#define DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_3_0__SHIFT		4
static inline uint32_t DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_3_0(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_3_0__SHIFT) & DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_3_0__MASK;
}
#define DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_7_4__MASK		0x000000f0
#define DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_7_4__SHIFT		4
static inline uint32_t DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_7_4(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_7_4__SHIFT) & DSI_14nm_PHY_CMN_CLK_CFG0_DIV_CTRL_7_4__MASK;
}

#define REG_DSI_14nm_PHY_CMN_CLK_CFG1				0x00000014
#define DSI_14nm_PHY_CMN_CLK_CFG1_DSICLK_SEL			0x00000001

#define REG_DSI_14nm_PHY_CMN_GLBL_TEST_CTRL			0x00000018
#define DSI_14nm_PHY_CMN_GLBL_TEST_CTRL_BITCLK_HS_SEL		0x00000004

#define REG_DSI_14nm_PHY_CMN_CTRL_0				0x0000001c

#define REG_DSI_14nm_PHY_CMN_CTRL_1				0x00000020

#define REG_DSI_14nm_PHY_CMN_HW_TRIGGER				0x00000024

#define REG_DSI_14nm_PHY_CMN_SW_CFG0				0x00000028

#define REG_DSI_14nm_PHY_CMN_SW_CFG1				0x0000002c

#define REG_DSI_14nm_PHY_CMN_SW_CFG2				0x00000030

#define REG_DSI_14nm_PHY_CMN_HW_CFG0				0x00000034

#define REG_DSI_14nm_PHY_CMN_HW_CFG1				0x00000038

#define REG_DSI_14nm_PHY_CMN_HW_CFG2				0x0000003c

#define REG_DSI_14nm_PHY_CMN_HW_CFG3				0x00000040

#define REG_DSI_14nm_PHY_CMN_HW_CFG4				0x00000044

#define REG_DSI_14nm_PHY_CMN_PLL_CNTRL				0x00000048
#define DSI_14nm_PHY_CMN_PLL_CNTRL_PLL_START			0x00000001

#define REG_DSI_14nm_PHY_CMN_LDO_CNTRL				0x0000004c
#define DSI_14nm_PHY_CMN_LDO_CNTRL_VREG_CTRL__MASK		0x0000003f
#define DSI_14nm_PHY_CMN_LDO_CNTRL_VREG_CTRL__SHIFT		0
static inline uint32_t DSI_14nm_PHY_CMN_LDO_CNTRL_VREG_CTRL(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_CMN_LDO_CNTRL_VREG_CTRL__SHIFT) & DSI_14nm_PHY_CMN_LDO_CNTRL_VREG_CTRL__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN(uint32_t i0) { return 0x00000000 + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_CFG0(uint32_t i0) { return 0x00000000 + 0x80*i0; }
#define DSI_14nm_PHY_LN_CFG0_PREPARE_DLY__MASK			0x000000c0
#define DSI_14nm_PHY_LN_CFG0_PREPARE_DLY__SHIFT			6
static inline uint32_t DSI_14nm_PHY_LN_CFG0_PREPARE_DLY(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_CFG0_PREPARE_DLY__SHIFT) & DSI_14nm_PHY_LN_CFG0_PREPARE_DLY__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_CFG1(uint32_t i0) { return 0x00000004 + 0x80*i0; }
#define DSI_14nm_PHY_LN_CFG1_HALFBYTECLK_EN			0x00000001

static inline uint32_t REG_DSI_14nm_PHY_LN_CFG2(uint32_t i0) { return 0x00000008 + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_CFG3(uint32_t i0) { return 0x0000000c + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_TEST_DATAPATH(uint32_t i0) { return 0x00000010 + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_TEST_STR(uint32_t i0) { return 0x00000014 + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_4(uint32_t i0) { return 0x00000018 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_4_HS_EXIT__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_4_HS_EXIT__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_4_HS_EXIT(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_4_HS_EXIT__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_4_HS_EXIT__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_5(uint32_t i0) { return 0x0000001c + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_5_HS_ZERO__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_5_HS_ZERO__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_5_HS_ZERO(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_5_HS_ZERO__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_5_HS_ZERO__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_6(uint32_t i0) { return 0x00000020 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_6_HS_PREPARE__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_6_HS_PREPARE__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_6_HS_PREPARE(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_6_HS_PREPARE__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_6_HS_PREPARE__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_7(uint32_t i0) { return 0x00000024 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_7_HS_TRAIL__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_7_HS_TRAIL__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_7_HS_TRAIL(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_7_HS_TRAIL__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_7_HS_TRAIL__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_8(uint32_t i0) { return 0x00000028 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_8_HS_RQST__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_8_HS_RQST__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_8_HS_RQST(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_8_HS_RQST__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_8_HS_RQST__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_9(uint32_t i0) { return 0x0000002c + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_GO__MASK		0x00000007
#define DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_GO__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_GO(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_GO__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_GO__MASK;
}
#define DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_SURE__MASK		0x00000070
#define DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_SURE__SHIFT		4
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_SURE(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_SURE__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_9_TA_SURE__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_10(uint32_t i0) { return 0x00000030 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_10_TA_GET__MASK		0x00000007
#define DSI_14nm_PHY_LN_TIMING_CTRL_10_TA_GET__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_10_TA_GET(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_10_TA_GET__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_10_TA_GET__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_TIMING_CTRL_11(uint32_t i0) { return 0x00000034 + 0x80*i0; }
#define DSI_14nm_PHY_LN_TIMING_CTRL_11_TRIG3_CMD__MASK		0x000000ff
#define DSI_14nm_PHY_LN_TIMING_CTRL_11_TRIG3_CMD__SHIFT		0
static inline uint32_t DSI_14nm_PHY_LN_TIMING_CTRL_11_TRIG3_CMD(uint32_t val)
{
	return ((val) << DSI_14nm_PHY_LN_TIMING_CTRL_11_TRIG3_CMD__SHIFT) & DSI_14nm_PHY_LN_TIMING_CTRL_11_TRIG3_CMD__MASK;
}

static inline uint32_t REG_DSI_14nm_PHY_LN_STRENGTH_CTRL_0(uint32_t i0) { return 0x00000038 + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_STRENGTH_CTRL_1(uint32_t i0) { return 0x0000003c + 0x80*i0; }

static inline uint32_t REG_DSI_14nm_PHY_LN_VREG_CNTRL(uint32_t i0) { return 0x00000064 + 0x80*i0; }

#define REG_DSI_14nm_PHY_PLL_IE_TRIM				0x00000000

#define REG_DSI_14nm_PHY_PLL_IP_TRIM				0x00000004

#define REG_DSI_14nm_PHY_PLL_IPTAT_TRIM				0x00000010

#define REG_DSI_14nm_PHY_PLL_CLKBUFLR_EN			0x0000001c

#define REG_DSI_14nm_PHY_PLL_SYSCLK_EN_RESET			0x00000028

#define REG_DSI_14nm_PHY_PLL_RESETSM_CNTRL			0x0000002c

#define REG_DSI_14nm_PHY_PLL_RESETSM_CNTRL2			0x00000030

#define REG_DSI_14nm_PHY_PLL_RESETSM_CNTRL3			0x00000034

#define REG_DSI_14nm_PHY_PLL_RESETSM_CNTRL4			0x00000038

#define REG_DSI_14nm_PHY_PLL_RESETSM_CNTRL5			0x0000003c

#define REG_DSI_14nm_PHY_PLL_KVCO_DIV_REF1			0x00000040

#define REG_DSI_14nm_PHY_PLL_KVCO_DIV_REF2			0x00000044

#define REG_DSI_14nm_PHY_PLL_KVCO_COUNT1			0x00000048

#define REG_DSI_14nm_PHY_PLL_KVCO_COUNT2			0x0000004c

#define REG_DSI_14nm_PHY_PLL_VREF_CFG1				0x0000005c

#define REG_DSI_14nm_PHY_PLL_KVCO_CODE				0x00000058

#define REG_DSI_14nm_PHY_PLL_VCO_DIV_REF1			0x0000006c

#define REG_DSI_14nm_PHY_PLL_VCO_DIV_REF2			0x00000070

#define REG_DSI_14nm_PHY_PLL_VCO_COUNT1				0x00000074

#define REG_DSI_14nm_PHY_PLL_VCO_COUNT2				0x00000078

#define REG_DSI_14nm_PHY_PLL_PLLLOCK_CMP1			0x0000007c

#define REG_DSI_14nm_PHY_PLL_PLLLOCK_CMP2			0x00000080

#define REG_DSI_14nm_PHY_PLL_PLLLOCK_CMP3			0x00000084

#define REG_DSI_14nm_PHY_PLL_PLLLOCK_CMP_EN			0x00000088

#define REG_DSI_14nm_PHY_PLL_PLL_VCO_TUNE			0x0000008c

#define REG_DSI_14nm_PHY_PLL_DEC_START				0x00000090

#define REG_DSI_14nm_PHY_PLL_SSC_EN_CENTER			0x00000094

#define REG_DSI_14nm_PHY_PLL_SSC_ADJ_PER1			0x00000098

#define REG_DSI_14nm_PHY_PLL_SSC_ADJ_PER2			0x0000009c

#define REG_DSI_14nm_PHY_PLL_SSC_PER1				0x000000a0

#define REG_DSI_14nm_PHY_PLL_SSC_PER2				0x000000a4

#define REG_DSI_14nm_PHY_PLL_SSC_STEP_SIZE1			0x000000a8

#define REG_DSI_14nm_PHY_PLL_SSC_STEP_SIZE2			0x000000ac

#define REG_DSI_14nm_PHY_PLL_DIV_FRAC_START1			0x000000b4

#define REG_DSI_14nm_PHY_PLL_DIV_FRAC_START2			0x000000b8

#define REG_DSI_14nm_PHY_PLL_DIV_FRAC_START3			0x000000bc

#define REG_DSI_14nm_PHY_PLL_TXCLK_EN				0x000000c0

#define REG_DSI_14nm_PHY_PLL_PLL_CRCTRL				0x000000c4

#define REG_DSI_14nm_PHY_PLL_RESET_SM_READY_STATUS		0x000000cc

#define REG_DSI_14nm_PHY_PLL_PLL_MISC1				0x000000e8

#define REG_DSI_14nm_PHY_PLL_CP_SET_CUR				0x000000f0

#define REG_DSI_14nm_PHY_PLL_PLL_ICPMSET			0x000000f4

#define REG_DSI_14nm_PHY_PLL_PLL_ICPCSET			0x000000f8

#define REG_DSI_14nm_PHY_PLL_PLL_ICP_SET			0x000000fc

#define REG_DSI_14nm_PHY_PLL_PLL_LPF1				0x00000100

#define REG_DSI_14nm_PHY_PLL_PLL_LPF2_POSTDIV			0x00000104

#define REG_DSI_14nm_PHY_PLL_PLL_BANDGAP			0x00000108


#endif /* DSI_PHY_14NM_XML */
