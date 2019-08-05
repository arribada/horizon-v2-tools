// test_prepas.cpp - Prepas unit tests
//
// Copyright (C) 2019 Arribada
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

extern "C"

{
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include "prepas.h"
}

#include <gtest/gtest.h>

class Test_prepas : public ::testing::Test
{
public:
    FILE	*ul_conf;   	/* configuration file	*/
    FILE	*ul_bull;   	/* orbit file 		*/
    FILE	*ul_resu;   	/* result file 		*/

    int	Nsat;					/* number of satellites in orbit file 	*/
    int	status;

    pc	tab_PC[1];              /* array of configuration parameter	*/
    po	tab_PO[8];              /* array of orbit parameter		*/
    //pp	tab_PP[1];              /* array of prediction parameters	*/

    po	*pt_po;                 /* pointer on tab_po			*/
    pc	*pt_pc;                 /* pointer on tab_pc			*/
    pp	*pt_pp;                 /* pointer on tab_pp			*/

    int	isat;

    int i;

    std::string nf_conf = "../prepas/prepas.cfg";
    std::string nf_bull = "../prepas/bulletin.dat";

    //char	message [256];
    char	ligne [MAXLU];

    int	tsat[8];		/* Order of satellite in orbit file	*/
    char	csat[2];

    //long	s_pp;		/* beginning of prediction (sec) */
    struct bulletin_data bulletin[8];
    uint8_t number_sat;
    float lon;
    float lat;
    uint32_t minimum_time;
    uint32_t desired_time;
    struct tm t_start, t_fin;
    uint32_t t_of_day_start, t_of_day_fin;

    void SetUp() override
    {
        /* Clear Orbit Paramater */
        for (isat = 0; isat < 8; isat++)
        {
            strcpy(tab_PO[isat].sat, "  ");
        }

        /*-------- Read Data Configuration ---------*/
        ul_conf = fopen(nf_conf.c_str(), "r");
        ASSERT_TRUE(ul_conf);
        lat = 52;
        lon = 1;
        t_of_day_start = 1552586400; // 14/04/2019 @ 6:00pm (UTC)

        /*-------- Read Orbit Parameter ---------*/
        struct tm t_bul;
        ul_bull = fopen(nf_bull.c_str(), "r");
        ASSERT_TRUE(ul_bull);

        time_t conver_time;
        fgets(ligne, MAXLU, ul_bull);
        sscanf(ligne, "%s%ld%f%f%f%f%f%f",
               &bulletin[0].sat,	&bulletin[0].time_bulletin,
               &bulletin[0].params[0],
               &bulletin[0].params[1],		&bulletin[0].params[2],
               &bulletin[0].params[3], 	&bulletin[0].params[4],
               &bulletin[0].params[5]);

        isat = 0;

        while (!feof(ul_bull))
        {

            isat++;
            fgets(ligne, MAXLU, ul_bull);
            strcpy(csat, "  ");
            sscanf(ligne, "%s%ld%f%f%f%f%f%f",
                   &bulletin[isat].sat,	&bulletin[isat].time_bulletin,
                   &bulletin[isat].params[0],
                   &bulletin[isat].params[1],		&bulletin[isat].params[2],
                   &bulletin[isat].params[3], 	&bulletin[isat].params[4],
                   &bulletin[isat].params[5]);
        } /* lecture de chaque bulletin de satellite */
        /* END WHILE */

        fclose (ul_bull);

        number_sat = isat;
    }
};

TEST_F(Test_prepas, SimplePrediction_1)
{
    uint32_t time = next_predict(bulletin, number_sat, lon, lat, t_of_day_start);
    EXPECT_EQ(time , 1552590810); //03/14/2019 @ 7:13pm (UTC)
}

TEST_F(Test_prepas, SimplePrediction_2)
{
    t_of_day_start = 1555261200; // 14/04/2019 @ 6:00pm (UTC)
    uint32_t time = next_predict(bulletin, number_sat, lon, lat, t_of_day_start);
    EXPECT_EQ(time , 1555263060);
}

TEST_F(Test_prepas, SimplePrediction_3_month)
{
    t_of_day_start = 1560531600; // 14/06/2019 @ 6:00pm (UTC)
    uint32_t time = next_predict(bulletin, number_sat, lon, lat, t_of_day_start);
    EXPECT_EQ(time , 1560537495);
}

TEST_F(Test_prepas, SimplePrediction_6_month)
{
    t_of_day_start = 1568480400; // 14/09/2019 @ 6:00pm (UTC)
    uint32_t time = next_predict(bulletin, number_sat, lon, lat, t_of_day_start);
    EXPECT_EQ(time , 1568485365);
}

TEST_F(Test_prepas, SimplePrediction_12_month)
{
    t_of_day_start = 1584208800; // 14/03/2020 @ 6:00pm (UTC)
    uint32_t time = next_predict(bulletin, number_sat, lon, lat, t_of_day_start);
    EXPECT_EQ(time , 1584210630);
}

TEST_F(Test_prepas, comp_original )
{
    lon = 1;
    lat = 52;
    t_of_day_start = 1552586400; // 14/04/2019 @ 6:00pm (UTC)#
    long minimum_time = t_of_day_start;
    pc	tab_PC[1];              /* array of configuration parameter	*/
    po	tab_PO[number_sat];
    pp tab_PP[number_sat];	      /* array of result */
    pp ref_tab_PP[number_sat];
    tab_PC[0].pf_lon = lon;
    tab_PC[0].pf_lat = lat;

    tab_PC[0].time_start = minimum_time;
    tab_PC[0].time_end = minimum_time + (48 * 60 * 60);
    tab_PC[0].s_differe = 0;

    tab_PC[0].site_min_requis = 45.0f;
    tab_PC[0].site_max_requis = 90.0f;

    tab_PC[0].marge_temporelle = 0;
    tab_PC[0].marge_geog_lat = 0;
    tab_PC[0].marge_geog_lon = 0;

    tab_PC[0].Npass_max = 1;

    po	*pt_po;                 /* pointer on tab_po			*/
    pc	*pt_pc;                 /* pointer on tab_pc			*/
    pp	*pt_pp;                 /* pointer on tab_pp			*/
    pp *ref_pt_pp;
    for (int i = 0; i < number_sat; ++i)
    {
        memcpy(tab_PO[i].sat, bulletin[i].sat, 2);
        tab_PO[i].time_bul = bulletin[i].time_bulletin;
        tab_PO[i].dga = bulletin[i].params[0];
        tab_PO[i].inc = bulletin[i].params[1];
        tab_PO[i].lon_asc = bulletin[i].params[2];
        tab_PO[i].d_noeud = bulletin[i].params[3];
        tab_PO[i].ts = bulletin[i].params[4];
        tab_PO[i].dgap = bulletin[i].params[5];
    }

    pt_pc  = &tab_PC[0];
    pt_po  = &tab_PO[0];
    pt_pp  = &tab_PP[0];
    ref_pt_pp  = &ref_tab_PP[0];

    //Load REF VALUES
    memcpy(ref_pt_pp[0].sat, "MA", 2);
    ref_pt_pp[0].tpp = 1552641863;		/* date du prochain passage (sec90) */
    ref_pt_pp[0].duree = 3;		/* duree (sec) */
    ref_pt_pp[0].site_max = 59;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[1].sat, "MB", 2);
    ref_pt_pp[1].tpp = 1552596204;		/* date du prochain passage (sec90) */
    ref_pt_pp[1].duree = 3;		/* duree (sec) */
    ref_pt_pp[1].site_max = 83;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[2].sat, "MC", 2);
    ref_pt_pp[2].tpp = 1552594224;		/* date du prochain passage (sec90) */
    ref_pt_pp[2].duree = 2;		/* duree (sec) */
    ref_pt_pp[2].site_max = 55;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[3].sat, "15", 2);
    ref_pt_pp[3].tpp = 1552633013;		/* date du prochain passage (sec90) */
    ref_pt_pp[3].duree = 2;		/* duree (sec) */
    ref_pt_pp[3].site_max = 48;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[4].sat, "18", 2);
    ref_pt_pp[4].tpp = 1552590714;		/* date du prochain passage (sec90) */
    ref_pt_pp[4].duree = 3;		/* duree (sec) */
    ref_pt_pp[4].site_max = 60;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[5].sat, "19", 2);
    ref_pt_pp[5].tpp = 1552626713;		/* date du prochain passage (sec90) */
    ref_pt_pp[5].duree = 3;		/* duree (sec) */
    ref_pt_pp[5].site_max = 74;	/* site max dans le passage (deg) */

    memcpy(ref_pt_pp[6].sat, "SR", 2);
    ref_pt_pp[6].tpp = 1552627223;		/* date du prochain passage (sec90) */
    ref_pt_pp[6].duree = 3;		/* duree (sec) */
    ref_pt_pp[6].site_max = 73;	/* site max dans le passage (deg) */

    prepas(pt_pc, pt_po, pt_pp, number_sat);

    for (int i = 0; i < number_sat; ++i)
    {
        EXPECT_EQ(pt_pp[i].sat[0] , ref_pt_pp[i].sat[0]);
        EXPECT_EQ(pt_pp[i].sat[1] , ref_pt_pp[i].sat[1]);
        EXPECT_EQ(pt_pp[i].tpp , ref_pt_pp[i].tpp);
        EXPECT_EQ(pt_pp[i].duree / 60 , ref_pt_pp[i].duree);
        EXPECT_EQ(pt_pp[i].site_max , ref_pt_pp[i].site_max);
    }
}