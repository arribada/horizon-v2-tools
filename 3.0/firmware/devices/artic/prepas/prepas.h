#define MIN(A,B) ((A)<(B)?(A):(B))
#define MAX(A,B) ((A)>(B)?(A):(B))
#define abs(n)   ((n)>=0?(n):(-n))

#define MAXLU 132
#define sortie_erreur -1

typedef struct
{
float		pf_lon;		/* geodetic position of the beacon */
float		pf_lat;		/* geodetic position of the beacon */
long time_start;
long time_end;
int s_differe;
float		site_min_requis;/* min site pour calcul de la duree (deg.) */
float		site_max_requis;/* min site (deg.) */	
float		marge_temporelle;/* marge temporelle (min/6mois) */
float		marge_geog_lat; /* marge geographique (deg) */
float		marge_geog_lon; /* marge geographique (deg) */
int		Npass_max;	/* nombre de passages max par satellite */
}pc;

typedef struct
{
char		sat[2];
long		time_bul;
float		dga;		/* semi-major axis (km) */
float		inc;		/* orbit inclination (deg) */
float		lon_asc;	/* longitude of ascending node (deg) */
float		d_noeud;	/* asc. node drift during one revolution (deg) */
float		ts;		/* orbit period (min) */
float		dgap;		/* drift of semi-major axis (m/jour) */
}po;

typedef struct
{
char		sat[2];
long		tpp;		/* date du prochain passage (sec90) */
int		duree;		/* duree (sec) */
int		site_max;	/* site max dans le passage (deg) */
}pp;


struct bulletin_data{
	char sat[2];
	float params[6];
	uint32_t time_bulletin;
};


uint32_t next_predict(struct bulletin_data *bulletin,  uint8_t number_sat,float lon, float lat, long minimum_time);
int prepas (pc * p_pc, po * p_po,		pp * p_pp,	int number_sat);
