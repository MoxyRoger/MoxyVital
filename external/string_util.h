/*
Copyright Fortiori Design, LLC   2013
P/N 2000195
Larry Flessland

*/

#ifndef STRING_UTIL_H
#define STRING_UTIL_H

extern int    STR_CharToInt(char *str, int *val);
extern int    STR_itoa(int *val, char *str);
extern int    STR_long_toa(long *val, char *str);
//extern void  *STR_memcpy(void* dest, const void* src, unsigned int count);
extern int    STR_scanf(char *StrIn, char* StrOut0, char* StrOut1, char* StrOut2, char* StrOut3, char* StrOut4);
extern void   STR_strcat(char *dest, char *source);
extern int    STR_strcmp(char *test1, char *test2);
extern void   STR_strcpy(char *dest, char *source);
extern int    STR_strlen(char *str);



#endif

