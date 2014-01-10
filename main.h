#ifdef MAIN_SOURCE
	#undef MAIN_SOURCE
	#define _INC 
	/* intern only */
	void init(void);
	#else	
	#define _INC extern
#endif

/* shared globals */

_INC volatile status_t stepstat ;

_INC volatile systemcfg_t systemconfig ;

_INC long val1t;

#undef _INC
