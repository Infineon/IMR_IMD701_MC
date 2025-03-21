/******************************************************************************
 * (c) (2024), Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product").
 * By including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing so
 * agrees to indemnify Cypress against all liability.
******************************************************************************/
/*
 * CORDIC_math.c
 *
 *  Created on: Apr 7, 2023
 *      Author: leeyoung
 */
#include "DataTypes.h"
#include "CORDIC_math.h"


const int16_t Cos_Sin_Q12[COS_SIN_Q12_TBL_SZ][2] = {
{4095	,0      },
{4095	,25     },
{4095	,50     },
{4094	,75     },
{4094	,100    },
{4093	,126    },
{4092	,151    },
{4091	,176    },
{4090	,201    },
{4089	,226    },
{4087	,251    },
{4086	,276    },
{4084	,301    },
{4082	,326    },
{4080	,351    },
{4078	,376    },
{4075	,401    },
{4073	,426    },
{4070	,451    },
{4067	,476    },
{4064	,501    },
{4061	,526    },
{4058	,551    },
{4054	,576    },
{4051	,601    },
{4047	,626    },
{4043	,651    },
{4039	,675    },
{4035	,700    },
{4030	,725    },
{4026	,750    },
{4021	,774    },
{4016	,799    },
{4011	,824    },
{4006	,848    },
{4001	,873    },
{3996	,897    },
{3990	,922    },
{3984	,946    },
{3978	,971    },
{3972	,995    },
{3966	,1019   },
{3960	,1044   },
{3953	,1068   },
{3947	,1092   },
{3940	,1116   },
{3933	,1141   },
{3926	,1165   },
{3919	,1189   },
{3911	,1213   },
{3904	,1237   },
{3896	,1261   },
{3888	,1285   },
{3880	,1308   },
{3872	,1332   },
{3864	,1356   },
{3856	,1380   },
{3847	,1403   },
{3838	,1427   },
{3830	,1450   },
{3821	,1474   },
{3811	,1497   },
{3802	,1521   },
{3793	,1544   },
{3783	,1567   },
{3774	,1590   },
{3764	,1613   },
{3754	,1636   },
{3744	,1659   },
{3733	,1682   },
{3723	,1705   },
{3713	,1728   },
{3702	,1751   },
{3691	,1774   },
{3680	,1796   },
{3669	,1819   },
{3658	,1841   },
{3646	,1864   },
{3635	,1886   },
{3623	,1908   },
{3611	,1930   },
{3600	,1952   },
{3588	,1975   },
{3575	,1997   },
{3563	,2018   },
{3551	,2040   },
{3538	,2062   },
{3525	,2084   },
{3512	,2105   },
{3499	,2127   },
{3486	,2148   },
{3473	,2170   },
{3460	,2191   },
{3446	,2212   },
{3433	,2233   },
{3419	,2254   },
{3405	,2275   },
{3391	,2296   },
{3377	,2317   },
{3362	,2337   },
{3348	,2358   },
{3333	,2378   },
{3319	,2399   },
{3304	,2419   },
{3289	,2439   },
{3274	,2460   },
{3259	,2480   },
{3244	,2500   },
{3228	,2519   },
{3213	,2539   },
{3197	,2559   },
{3181	,2578   },
{3165	,2598   },
{3149	,2617   },
{3133	,2636   },
{3117	,2656   },
{3101	,2675   },
{3084	,2694   },
{3068	,2713   },
{3051	,2731   },
{3034	,2750   },
{3017	,2769   },
{3000	,2787   },
{2983	,2805   },
{2966	,2824   },
{2948	,2842   },
{2931	,2860   },
{2913	,2878   },
{2896	,2896   },
{2878	,2913   },
{2860	,2931   },
{2842	,2948   },
{2824	,2966   },
{2805	,2983   },
{2787	,3000   },
{2769	,3017   },
{2750	,3034   },
{2731	,3051   },
{2713	,3068   },
{2694	,3084   },
{2675	,3101   },
{2656	,3117   },
{2636	,3133   },
{2617	,3149   },
{2598	,3165   },
{2578	,3181   },
{2559	,3197   },
{2539	,3213   },
{2519	,3228   },
{2500	,3244   },
{2480	,3259   },
{2460	,3274   },
{2439	,3289   },
{2419	,3304   },
{2399	,3319   },
{2378	,3333   },
{2358	,3348   },
{2337	,3362   },
{2317	,3377   },
{2296	,3391   },
{2275	,3405   },
{2254	,3419   },
{2233	,3433   },
{2212	,3446   },
{2191	,3460   },
{2170	,3473   },
{2148	,3486   },
{2127	,3499   },
{2105	,3512   },
{2084	,3525   },
{2062	,3538   },
{2040	,3551   },
{2018	,3563   },
{1997	,3575   },
{1975	,3588   },
{1952	,3600   },
{1930	,3611   },
{1908	,3623   },
{1886	,3635   },
{1864	,3646   },
{1841	,3658   },
{1819	,3669   },
{1796	,3680   },
{1774	,3691   },
{1751	,3702   },
{1728	,3713   },
{1705	,3723   },
{1682	,3733   },
{1659	,3744   },
{1636	,3754   },
{1613	,3764   },
{1590	,3774   },
{1567	,3783   },
{1544	,3793   },
{1521	,3802   },
{1497	,3811   },
{1474	,3821   },
{1450	,3830   },
{1427	,3838   },
{1403	,3847   },
{1380	,3856   },
{1356	,3864   },
{1332	,3872   },
{1308	,3880   },
{1285	,3888   },
{1261	,3896   },
{1237	,3904   },
{1213	,3911   },
{1189	,3919   },
{1165	,3926   },
{1141	,3933   },
{1116	,3940   },
{1092	,3947   },
{1068	,3953   },
{1044	,3960   },
{1019	,3966   },
{995	,3972   },
{971	,3978   },
{946	,3984   },
{922	,3990   },
{897	,3996   },
{873	,4001   },
{848	,4006   },
{824	,4011   },
{799	,4016   },
{774	,4021   },
{750	,4026   },
{725	,4030   },
{700	,4035   },
{675	,4039   },
{651	,4043   },
{626	,4047   },
{601	,4051   },
{576	,4054   },
{551	,4058   },
{526	,4061   },
{501	,4064   },
{476	,4067   },
{451	,4070   },
{426	,4073   },
{401	,4075   },
{376	,4078   },
{351	,4080   },
{326	,4082   },
{301	,4084   },
{276	,4086   },
{251	,4087   },
{226	,4089   },
{201	,4090   },
{176	,4091   },
{151	,4092   },
{126	,4093   },
{100	,4094   },
{75	    ,4094   },
{50	    ,4095   },
{25	    ,4095   },
{0	    ,4095   },
{-25	,4095   },
{-50	,4095   },
{-75	,4094   },
{-100	,4094   },
{-126	,4093   },
{-151	,4092   },
{-176	,4091   },
{-201	,4090   },
{-226	,4089   },
{-251	,4087   },
{-276	,4086   },
{-301	,4084   },
{-326	,4082   },
{-351	,4080   },
{-376	,4078   },
{-401	,4075   },
{-426	,4073   },
{-451	,4070   },
{-476	,4067   },
{-501	,4064   },
{-526	,4061   },
{-551	,4058   },
{-576	,4054   },
{-601	,4051   },
{-626	,4047   },
{-651	,4043   },
{-675	,4039   },
{-700	,4035   },
{-725	,4030   },
{-750	,4026   },
{-774	,4021   },
{-799	,4016   },
{-824	,4011   },
{-848	,4006   },
{-873	,4001   },
{-897	,3996   },
{-922	,3990   },
{-946	,3984   },
{-971	,3978   },
{-995	,3972   },
{-1019	,3966   },
{-1044	,3960   },
{-1068	,3953   },
{-1092	,3947   },
{-1116	,3940   },
{-1141	,3933   },
{-1165	,3926   },
{-1189	,3919   },
{-1213	,3911   },
{-1237	,3904   },
{-1261	,3896   },
{-1285	,3888   },
{-1308	,3880   },
{-1332	,3872   },
{-1356	,3864   },
{-1380	,3856   },
{-1403	,3847   },
{-1427	,3838   },
{-1450	,3830   },
{-1474	,3821   },
{-1497	,3811   },
{-1521	,3802   },
{-1544	,3793   },
{-1567	,3783   },
{-1590	,3774   },
{-1613	,3764   },
{-1636	,3754   },
{-1659	,3744   },
{-1682	,3733   },
{-1705	,3723   },
{-1728	,3713   },
{-1751	,3702   },
{-1774	,3691   },
{-1796	,3680   },
{-1819	,3669   },
{-1841	,3658   },
{-1864	,3646   },
{-1886	,3635   },
{-1908	,3623   },
{-1930	,3611   },
{-1952	,3600   },
{-1975	,3588   },
{-1997	,3575   },
{-2018	,3563   },
{-2040	,3551   },
{-2062	,3538   },
{-2084	,3525   },
{-2105	,3512   },
{-2127	,3499   },
{-2148	,3486   },
{-2170	,3473   },
{-2191	,3460   },
{-2212	,3446   },
{-2233	,3433   },
{-2254	,3419   },
{-2275	,3405   },
{-2296	,3391   },
{-2317	,3377   },
{-2337	,3362   },
{-2358	,3348   },
{-2378	,3333   },
{-2399	,3319   },
{-2419	,3304   },
{-2439	,3289   },
{-2460	,3274   },
{-2480	,3259   },
{-2500	,3244   },
{-2519	,3228   },
{-2539	,3213   },
{-2559	,3197   },
{-2578	,3181   },
{-2598	,3165   },
{-2617	,3149   },
{-2636	,3133   },
{-2656	,3117   },
{-2675	,3101   },
{-2694	,3084   },
{-2713	,3068   },
{-2731	,3051   },
{-2750	,3034   },
{-2769	,3017   },
{-2787	,3000   },
{-2805	,2983   },
{-2824	,2966   },
{-2842	,2948   },
{-2860	,2931   },
{-2878	,2913   },
{-2896	,2896   },
{-2913	,2878   },
{-2931	,2860   },
{-2948	,2842   },
{-2966	,2824   },
{-2983	,2805   },
{-3000	,2787   },
{-3017	,2769   },
{-3034	,2750   },
{-3051	,2731   },
{-3068	,2713   },
{-3084	,2694   },
{-3101	,2675   },
{-3117	,2656   },
{-3133	,2636   },
{-3149	,2617   },
{-3165	,2598   },
{-3181	,2578   },
{-3197	,2559   },
{-3213	,2539   },
{-3228	,2519   },
{-3244	,2500   },
{-3259	,2480   },
{-3274	,2460   },
{-3289	,2439   },
{-3304	,2419   },
{-3319	,2399   },
{-3333	,2378   },
{-3348	,2358   },
{-3362	,2337   },
{-3377	,2317   },
{-3391	,2296   },
{-3405	,2275   },
{-3419	,2254   },
{-3433	,2233   },
{-3446	,2212   },
{-3460	,2191   },
{-3473	,2170   },
{-3486	,2148   },
{-3499	,2127   },
{-3512	,2105   },
{-3525	,2084   },
{-3538	,2062   },
{-3551	,2040   },
{-3563	,2018   },
{-3575	,1997   },
{-3588	,1975   },
{-3600	,1952   },
{-3611	,1930   },
{-3623	,1908   },
{-3635	,1886   },
{-3646	,1864   },
{-3658	,1841   },
{-3669	,1819   },
{-3680	,1796   },
{-3691	,1774   },
{-3702	,1751   },
{-3713	,1728   },
{-3723	,1705   },
{-3733	,1682   },
{-3744	,1659   },
{-3754	,1636   },
{-3764	,1613   },
{-3774	,1590   },
{-3783	,1567   },
{-3793	,1544   },
{-3802	,1521   },
{-3811	,1497   },
{-3821	,1474   },
{-3830	,1450   },
{-3838	,1427   },
{-3847	,1403   },
{-3856	,1380   },
{-3864	,1356   },
{-3872	,1332   },
{-3880	,1308   },
{-3888	,1285   },
{-3896	,1261   },
{-3904	,1237   },
{-3911	,1213   },
{-3919	,1189   },
{-3926	,1165   },
{-3933	,1141   },
{-3940	,1116   },
{-3947	,1092   },
{-3953	,1068   },
{-3960	,1044   },
{-3966	,1019   },
{-3972	,995    },
{-3978	,971    },
{-3984	,946    },
{-3990	,922    },
{-3996	,897    },
{-4001	,873    },
{-4006	,848    },
{-4011	,824    },
{-4016	,799    },
{-4021	,774    },
{-4026	,750    },
{-4030	,725    },
{-4035	,700    },
{-4039	,675    },
{-4043	,651    },
{-4047	,626    },
{-4051	,601    },
{-4054	,576    },
{-4058	,551    },
{-4061	,526    },
{-4064	,501    },
{-4067	,476    },
{-4070	,451    },
{-4073	,426    },
{-4075	,401    },
{-4078	,376    },
{-4080	,351    },
{-4082	,326    },
{-4084	,301    },
{-4086	,276    },
{-4087	,251    },
{-4089	,226    },
{-4090	,201    },
{-4091	,176    },
{-4092	,151    },
{-4093	,126    },
{-4094	,100    },
{-4094	,75     },
{-4095	,50     },
{-4095	,25     },
{-4095	,0      },
{-4095	,-25    },
{-4095	,-50    },
{-4094	,-75    },
{-4094	,-100   },
{-4093	,-126   },
{-4092	,-151   },
{-4091	,-176   },
{-4090	,-201   },
{-4089	,-226   },
{-4087	,-251   },
{-4086	,-276   },
{-4084	,-301   },
{-4082	,-326   },
{-4080	,-351   },
{-4078	,-376   },
{-4075	,-401   },
{-4073	,-426   },
{-4070	,-451   },
{-4067	,-476   },
{-4064	,-501   },
{-4061	,-526   },
{-4058	,-551   },
{-4054	,-576   },
{-4051	,-601   },
{-4047	,-626   },
{-4043	,-651   },
{-4039	,-675   },
{-4035	,-700   },
{-4030	,-725   },
{-4026	,-750   },
{-4021	,-774   },
{-4016	,-799   },
{-4011	,-824   },
{-4006	,-848   },
{-4001	,-873   },
{-3996	,-897   },
{-3990	,-922   },
{-3984	,-946   },
{-3978	,-971   },
{-3972	,-995   },
{-3966	,-1019  },
{-3960	,-1044  },
{-3953	,-1068  },
{-3947	,-1092  },
{-3940	,-1116  },
{-3933	,-1141  },
{-3926	,-1165  },
{-3919	,-1189  },
{-3911	,-1213  },
{-3904	,-1237  },
{-3896	,-1261  },
{-3888	,-1285  },
{-3880	,-1308  },
{-3872	,-1332  },
{-3864	,-1356  },
{-3856	,-1380  },
{-3847	,-1403  },
{-3838	,-1427  },
{-3830	,-1450  },
{-3821	,-1474  },
{-3811	,-1497  },
{-3802	,-1521  },
{-3793	,-1544  },
{-3783	,-1567  },
{-3774	,-1590  },
{-3764	,-1613  },
{-3754	,-1636  },
{-3744	,-1659  },
{-3733	,-1682  },
{-3723	,-1705  },
{-3713	,-1728  },
{-3702	,-1751  },
{-3691	,-1774  },
{-3680	,-1796  },
{-3669	,-1819  },
{-3658	,-1841  },
{-3646	,-1864  },
{-3635	,-1886  },
{-3623	,-1908  },
{-3611	,-1930  },
{-3600	,-1952  },
{-3588	,-1975  },
{-3575	,-1997  },
{-3563	,-2018  },
{-3551	,-2040  },
{-3538	,-2062  },
{-3525	,-2084  },
{-3512	,-2105  },
{-3499	,-2127  },
{-3486	,-2148  },
{-3473	,-2170  },
{-3460	,-2191  },
{-3446	,-2212  },
{-3433	,-2233  },
{-3419	,-2254  },
{-3405	,-2275  },
{-3391	,-2296  },
{-3377	,-2317  },
{-3362	,-2337  },
{-3348	,-2358  },
{-3333	,-2378  },
{-3319	,-2399  },
{-3304	,-2419  },
{-3289	,-2439  },
{-3274	,-2460  },
{-3259	,-2480  },
{-3244	,-2500  },
{-3228	,-2519  },
{-3213	,-2539  },
{-3197	,-2559  },
{-3181	,-2578  },
{-3165	,-2598  },
{-3149	,-2617  },
{-3133	,-2636  },
{-3117	,-2656  },
{-3101	,-2675  },
{-3084	,-2694  },
{-3068	,-2713  },
{-3051	,-2731  },
{-3034	,-2750  },
{-3017	,-2769  },
{-3000	,-2787  },
{-2983	,-2805  },
{-2966	,-2824  },
{-2948	,-2842  },
{-2931	,-2860  },
{-2913	,-2878  },
{-2896	,-2896  },
{-2878	,-2913  },
{-2860	,-2931  },
{-2842	,-2948  },
{-2824	,-2966  },
{-2805	,-2983  },
{-2787	,-3000  },
{-2769	,-3017  },
{-2750	,-3034  },
{-2731	,-3051  },
{-2713	,-3068  },
{-2694	,-3084  },
{-2675	,-3101  },
{-2656	,-3117  },
{-2636	,-3133  },
{-2617	,-3149  },
{-2598	,-3165  },
{-2578	,-3181  },
{-2559	,-3197  },
{-2539	,-3213  },
{-2519	,-3228  },
{-2500	,-3244  },
{-2480	,-3259  },
{-2460	,-3274  },
{-2439	,-3289  },
{-2419	,-3304  },
{-2399	,-3319  },
{-2378	,-3333  },
{-2358	,-3348  },
{-2337	,-3362  },
{-2317	,-3377  },
{-2296	,-3391  },
{-2275	,-3405  },
{-2254	,-3419  },
{-2233	,-3433  },
{-2212	,-3446  },
{-2191	,-3460  },
{-2170	,-3473  },
{-2148	,-3486  },
{-2127	,-3499  },
{-2105	,-3512  },
{-2084	,-3525  },
{-2062	,-3538  },
{-2040	,-3551  },
{-2018	,-3563  },
{-1997	,-3575  },
{-1975	,-3588  },
{-1952	,-3600  },
{-1930	,-3611  },
{-1908	,-3623  },
{-1886	,-3635  },
{-1864	,-3646  },
{-1841	,-3658  },
{-1819	,-3669  },
{-1796	,-3680  },
{-1774	,-3691  },
{-1751	,-3702  },
{-1728	,-3713  },
{-1705	,-3723  },
{-1682	,-3733  },
{-1659	,-3744  },
{-1636	,-3754  },
{-1613	,-3764  },
{-1590	,-3774  },
{-1567	,-3783  },
{-1544	,-3793  },
{-1521	,-3802  },
{-1497	,-3811  },
{-1474	,-3821  },
{-1450	,-3830  },
{-1427	,-3838  },
{-1403	,-3847  },
{-1380	,-3856  },
{-1356	,-3864  },
{-1332	,-3872  },
{-1308	,-3880  },
{-1285	,-3888  },
{-1261	,-3896  },
{-1237	,-3904  },
{-1213	,-3911  },
{-1189	,-3919  },
{-1165	,-3926  },
{-1141	,-3933  },
{-1116	,-3940  },
{-1092	,-3947  },
{-1068	,-3953  },
{-1044	,-3960  },
{-1019	,-3966  },
{-995	,-3972  },
{-971	,-3978  },
{-946	,-3984  },
{-922	,-3990  },
{-897	,-3996  },
{-873	,-4001  },
{-848	,-4006  },
{-824	,-4011  },
{-799	,-4016  },
{-774	,-4021  },
{-750	,-4026  },
{-725	,-4030  },
{-700	,-4035  },
{-675	,-4039  },
{-651	,-4043  },
{-626	,-4047  },
{-601	,-4051  },
{-576	,-4054  },
{-551	,-4058  },
{-526	,-4061  },
{-501	,-4064  },
{-476	,-4067  },
{-451	,-4070  },
{-426	,-4073  },
{-401	,-4075  },
{-376	,-4078  },
{-351	,-4080  },
{-326	,-4082  },
{-301	,-4084  },
{-276	,-4086  },
{-251	,-4087  },
{-226	,-4089  },
{-201	,-4090  },
{-176	,-4091  },
{-151	,-4092  },
{-126	,-4093  },
{-100	,-4094  },
{-75	,-4094  },
{-50	,-4095  },
{-25	,-4095  },
{0	    ,-4095  },
{25	    ,-4095  },
{50	    ,-4095  },
{75	    ,-4094  },
{100	,-4094  },
{126	,-4093  },
{151	,-4092  },
{176	,-4091  },
{201	,-4090  },
{226	,-4089  },
{251	,-4087  },
{276	,-4086  },
{301	,-4084  },
{326	,-4082  },
{351	,-4080  },
{376	,-4078  },
{401	,-4075  },
{426	,-4073  },
{451	,-4070  },
{476	,-4067  },
{501	,-4064  },
{526	,-4061  },
{551	,-4058  },
{576	,-4054  },
{601	,-4051  },
{626	,-4047  },
{651	,-4043  },
{675	,-4039  },
{700	,-4035  },
{725	,-4030  },
{750	,-4026  },
{774	,-4021  },
{799	,-4016  },
{824	,-4011  },
{848	,-4006  },
{873	,-4001  },
{897	,-3996  },
{922	,-3990  },
{946	,-3984  },
{971	,-3978  },
{995	,-3972  },
{1019	,-3966  },
{1044	,-3960  },
{1068	,-3953  },
{1092	,-3947  },
{1116	,-3940  },
{1141	,-3933  },
{1165	,-3926  },
{1189	,-3919  },
{1213	,-3911  },
{1237	,-3904  },
{1261	,-3896  },
{1285	,-3888  },
{1308	,-3880  },
{1332	,-3872  },
{1356	,-3864  },
{1380	,-3856  },
{1403	,-3847  },
{1427	,-3838  },
{1450	,-3830  },
{1474	,-3821  },
{1497	,-3811  },
{1521	,-3802  },
{1544	,-3793  },
{1567	,-3783  },
{1590	,-3774  },
{1613	,-3764  },
{1636	,-3754  },
{1659	,-3744  },
{1682	,-3733  },
{1705	,-3723  },
{1728	,-3713  },
{1751	,-3702  },
{1774	,-3691  },
{1796	,-3680  },
{1819	,-3669  },
{1841	,-3658  },
{1864	,-3646  },
{1886	,-3635  },
{1908	,-3623  },
{1930	,-3611  },
{1952	,-3600  },
{1975	,-3588  },
{1997	,-3575  },
{2018	,-3563  },
{2040	,-3551  },
{2062	,-3538  },
{2084	,-3525  },
{2105	,-3512  },
{2127	,-3499  },
{2148	,-3486  },
{2170	,-3473  },
{2191	,-3460  },
{2212	,-3446  },
{2233	,-3433  },
{2254	,-3419  },
{2275	,-3405  },
{2296	,-3391  },
{2317	,-3377  },
{2337	,-3362  },
{2358	,-3348  },
{2378	,-3333  },
{2399	,-3319  },
{2419	,-3304  },
{2439	,-3289  },
{2460	,-3274  },
{2480	,-3259  },
{2500	,-3244  },
{2519	,-3228  },
{2539	,-3213  },
{2559	,-3197  },
{2578	,-3181  },
{2598	,-3165  },
{2617	,-3149  },
{2636	,-3133  },
{2656	,-3117  },
{2675	,-3101  },
{2694	,-3084  },
{2713	,-3068  },
{2731	,-3051  },
{2750	,-3034  },
{2769	,-3017  },
{2787	,-3000  },
{2805	,-2983  },
{2824	,-2966  },
{2842	,-2948  },
{2860	,-2931  },
{2878	,-2913  },
{2896	,-2896  },
{2913	,-2878  },
{2931	,-2860  },
{2948	,-2842  },
{2966	,-2824  },
{2983	,-2805  },
{3000	,-2787  },
{3017	,-2769  },
{3034	,-2750  },
{3051	,-2731  },
{3068	,-2713  },
{3084	,-2694  },
{3101	,-2675  },
{3117	,-2656  },
{3133	,-2636  },
{3149	,-2617  },
{3165	,-2598  },
{3181	,-2578  },
{3197	,-2559  },
{3213	,-2539  },
{3228	,-2519  },
{3244	,-2500  },
{3259	,-2480  },
{3274	,-2460  },
{3289	,-2439  },
{3304	,-2419  },
{3319	,-2399  },
{3333	,-2378  },
{3348	,-2358  },
{3362	,-2337  },
{3377	,-2317  },
{3391	,-2296  },
{3405	,-2275  },
{3419	,-2254  },
{3433	,-2233  },
{3446	,-2212  },
{3460	,-2191  },
{3473	,-2170  },
{3486	,-2148  },
{3499	,-2127  },
{3512	,-2105  },
{3525	,-2084  },
{3538	,-2062  },
{3551	,-2040  },
{3563	,-2018  },
{3575	,-1997  },
{3588	,-1975  },
{3600	,-1952  },
{3611	,-1930  },
{3623	,-1908  },
{3635	,-1886  },
{3646	,-1864  },
{3658	,-1841  },
{3669	,-1819  },
{3680	,-1796  },
{3691	,-1774  },
{3702	,-1751  },
{3713	,-1728  },
{3723	,-1705  },
{3733	,-1682  },
{3744	,-1659  },
{3754	,-1636  },
{3764	,-1613  },
{3774	,-1590  },
{3783	,-1567  },
{3793	,-1544  },
{3802	,-1521  },
{3811	,-1497  },
{3821	,-1474  },
{3830	,-1450  },
{3838	,-1427  },
{3847	,-1403  },
{3856	,-1380  },
{3864	,-1356  },
{3872	,-1332  },
{3880	,-1308  },
{3888	,-1285  },
{3896	,-1261  },
{3904	,-1237  },
{3911	,-1213  },
{3919	,-1189  },
{3926	,-1165  },
{3933	,-1141  },
{3940	,-1116  },
{3947	,-1092  },
{3953	,-1068  },
{3960	,-1044  },
{3966	,-1019  },
{3972	,-995   },
{3978	,-971   },
{3984	,-946   },
{3990	,-922   },
{3996	,-897   },
{4001	,-873   },
{4006	,-848   },
{4011	,-824   },
{4016	,-799   },
{4021	,-774   },
{4026	,-750   },
{4030	,-725   },
{4035	,-700   },
{4039	,-675   },
{4043	,-651   },
{4047	,-626   },
{4051	,-601   },
{4054	,-576   },
{4058	,-551   },
{4061	,-526   },
{4064	,-501   },
{4067	,-476   },
{4070	,-451   },
{4073	,-426   },
{4075	,-401   },
{4078	,-376   },
{4080	,-351   },
{4082	,-326   },
{4084	,-301   },
{4086	,-276   },
{4087	,-251   },
{4089	,-226   },
{4090	,-201   },
{4091	,-176   },
{4092	,-151   },
{4093	,-126   },
{4094	,-100   },
{4094	,-75    },
{4095	,-50    },
{4095	,-25    },
};

__RAM_FUNC void Get_CosSin_Q12(uint16_t ThetaU16, q_t *Cos, q_t *Sin)
{
	ThetaU16 = (ThetaU16 >> Q6) & COS_SIN_TBL_IDX_MASK;
	if (ThetaU16 <= 256)	// the 1st quadrant
	{
		*Cos = Cos_Sin_Q12[ThetaU16][0];
		*Sin = Cos_Sin_Q12[ThetaU16][1];
	}
	else if (ThetaU16 <= 512)
	{
		ThetaU16 = 512 - ThetaU16;
		*Cos = -Cos_Sin_Q12[ThetaU16][0];
		*Sin =  Cos_Sin_Q12[ThetaU16][1];
	}
	else if (ThetaU16 <= 768)
	{
		ThetaU16 -= 512;
		*Cos = -Cos_Sin_Q12[ThetaU16][0];
		*Sin = -Cos_Sin_Q12[ThetaU16][1];
	}
	else
	{
		ThetaU16 = 1023 - ThetaU16;
		*Cos =  Cos_Sin_Q12[ThetaU16][0];
		*Sin = -Cos_Sin_Q12[ThetaU16][1];
	}
}
