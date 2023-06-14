#pragma once
// #ifndef VALUE_REDEFINE
#include <iostream>
#include <stdio.h>
#include <stdint.h>

// #define VALUE_REDEFINE
/*!< Signed integer types  */
typedef   signed char     int8_t;
typedef   signed short    int16_t;

typedef int32_t  i32;
typedef int16_t i16;
typedef int8_t  i8;

//typedef   signed long     int32_t;
/*!< Unsigned integer types  */

typedef unsigned char     uint8_t;  
typedef unsigned short    uint16_t;

typedef uint32_t  u32; 
typedef uint16_t u16;
typedef uint8_t  u8;


typedef float  f32;
typedef double  f64;

//system word length
typedef long long  isize;    
typedef unsigned long long  usize;



// #endif
// printf("u8:%lld\n",sizeof(u8)*8);
// printf("u16:%lld\n",sizeof(u16)*8);
// printf("u32:%lld\n",sizeof(u32)*8);
// printf("i8:%lld\n",sizeof(i8)*8);
// printf("i16:%lld\n",sizeof(i16)*8);
// printf("i32:%lld\n",sizeof(i32)*8);
// printf("f32:%lld\n",sizeof(f32)*8);
// printf("f64:%lld\n",sizeof(f64)*8);
// printf("usize:%lld\n",sizeof(usize)*8);
// printf("isize:%lld\n",sizeof(isize)*8);

// u8:8
// u16:16
// u32:32
// i8:8
// i16:16
// i32:32
// f32:32
// f64:64
// usize:64
// isize:64

// 32:
// char：1
// char*:4
// shortint:2
// int：4
// unsignedint:4
// float:4
//double:8
// long:4
// longlong:8
// unsignedlong:4

// 64:
// char：1
// char*:8
// shortint:2
// int：4
// unsignedint:4
// float:4
// double:8
// long:8
// longlong:8
// unsignedlong:8