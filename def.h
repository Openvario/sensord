#define TRUE 1
#define FALSE 0

#define debug_print(...) if(g_debug>0)fprintf(fp_console,__VA_ARGS__)
#define ddebug_print(...) if(g_debug>1)fprintf(fp_console,__VA_ARGS__)

#define log(...) if(g_log>0)fprintf(__VA_ARGS__)