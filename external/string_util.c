
/*
Copyright Fortiori Design, LLC   2013
P/N 2000195
Larry Flessland

*/
#include "string_util.h"

//--------------------------------------------------
// Data structures
//--------------------------------------------------

//--------------------------------------------------
// Constants
//--------------------------------------------------

//--------------------------------------------------
// Function Prototypes
//--------------------------------------------------

//------------------------------------------------
int    STR_CharToInt(char *str, int *val)
{
  int len = 0;
  int i = 0;
  int done = 0;
  int ec = 0;

  len = STR_strlen(str);

  if (len > 0)
  {
    do
    {
      if ((str[i] < '0') || (str[i] > '9'))
      {
        ec = 1;
        done = 1;
      }
      i++;
    } while ((done == 0) && (i < len));

    i = 0;
    if (ec == 0)
    {
      while (*str)
      {
        i = (i << 3) + (i << 1) + (*str - '0');
        str++;
      }
    }
  }
  *val = i;
  return(ec);
}

//------------------------------------------------
int    STR_itoa(int *val, char *str)
{

  // todo: not tested.
  int num = 0;
  int i = 0;
  int j = 0;
  int len = 0;
  char TempStr[10] = "";
  int ec = 0;

  num = *val;
  STR_strcpy(str, "");

  if (num == 0)
  {
    STR_strcpy(TempStr, "0");
  }
  else
  {
    j = 0;
    while (num > 0)
    {
      TempStr[j] = (char) ((num % 10) + 48);  // 48=(int)'0';
      num /= 10;
      j++;
    }
    len = j;
    for (i = 0; i < len; i++)
    {
      str[i] = TempStr[len - i - 1];
    }
    str[len] = '\0';

  }
  return(ec);
}
//------------------------------------------------
int    STR_long_toa(long *val, char *str)
{

  // todo: not tested.
  long num = 0;
  int i = 0;
  int j = 0;
  int len = 0;
  int neg_flag = 0;
  char TempStr[21] = "";
  int ec = 0;

  num = *val;
  STR_strcpy(str, "");

  if (num == 0)
  {
    STR_strcpy(TempStr, "0");
  }
  else
  {
    if (num < 0)
    {
      neg_flag = 1;
      num = -num;
    }
      
    j = 0;
    while (num > 0)
    {
      TempStr[j] = (char) ((num % 10) + '0');  // 48=(int)'0';
      num /= 10;
      j++;
    }
    if (neg_flag != 0)
    {
      TempStr[j] = '-';
      j++;
    }
    len = j;
    for (i = 0; i < len; i++)
    {
      str[i] = TempStr[len - i - 1];
    }
    str[len] = '\0';

  }
  return(ec);
}

//------------------------------------------------
// void *STR_memcpy(void* dest, const void* src, unsigned int count)
// {
//   char* dst8 = (char*)dest;
//   char* src8 = (char*)src;

//   while (count--) {
//       *dst8++ = *src8++;
//   }
//   return dest;
// }

//------------------------------------------------
int    STR_scanf(char *StrIn, char* StrOut0, char* StrOut1, char* StrOut2, char* StrOut3, char* StrOut4)
{
  int args = 0;
  int len = 0;
  int i = 0;
  char PrevChar = '\0';
  char *ptrChar = '\0';
//
//  *StrOut0 = '\0';
//  *StrOut1 = '\0';
//  *StrOut2 = '\0';
//  *StrOut3 = '\0';
//  *StrOut4 = '\0';

  len = STR_strlen(StrIn);
  if (len == 0)
  {
    args = 0;
  }
  else
  {
    for (i = 0; i < len; i++)
    {
      if (StrIn[i] > ' ')   // check for valid string
      {
        //
        if (PrevChar <= ' ')
        {
          // point to next string
          switch (args)
          {
            case 0:
              ptrChar = StrOut0;
              break;
            case 1:
              ptrChar = StrOut1;
              break;
            case 2:
              ptrChar = StrOut2;
              break;
            case 3:
              ptrChar = StrOut3;
              break;
            case 4:
              ptrChar = StrOut4;
              break;
          }
          args++;            
        }
        *ptrChar = StrIn[i];  // transfer charactrer to new string
         ptrChar++;             // advance string pointer
        *ptrChar = '\0';       // terminate with Null
      }
      PrevChar = StrIn[i];
    }   
  
  }

  return(args);
}

//------------------------------------------------
void   STR_strcat(char *dest, char *source)
{
  int length = 0;
  
  length = STR_strlen(dest);
  if (length > 0) // todo: this can be deleted.
  {
 //   length = length;  // get rid of null terminator
  }
  
  STR_strcpy(dest + length, source);  // copy the source at the end of dest 
}
//------------------------------------------------
int    STR_strcmp(char *test1, char *test2)
{
  int i = 0;
  int len1 = 0;
  int len2 = 0;
  int ret = 0;

  len1 = STR_strlen(test1);
  len2 = STR_strlen(test2);

  if (len1 != len2)
  {
    ret = 1;
  }
  else
  {

    i = 0;
    do
    {
      if (test1[i] != test2[i])
      {
        ret = 2;
      }
      i++;
    } while ((ret == 0) && (i < len1));
  }

  return(ret);
}
//------------------------------------------------
void  STR_strcpy(char *dest, char *source)
{
  int i = 0;

  do
  {
    dest[i] = source[i];
    i++;
  } while (source[i] != '\0');
  dest[i] = '\0';   // Fill the last space with NULL
}

//------------------------------------------------
int   STR_strlen(char *str)
{
  int i = 0;

  do
  {
    if (str[i] != '\0')
    {
      i++;
    }
  } while (str[i] != '\0');

  return(i);
}

