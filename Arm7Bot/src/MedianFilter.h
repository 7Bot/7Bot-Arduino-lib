/****************************************************
/* 7Bot class for Arduino platform
/* Author: Jerry Peng
/* Date: 26 April 2016
/* 
/* Version 1.00
/* www.7bot.cc
/*  
/* Description: 
/* 
/*
/***************************************************/

#ifndef _MEDIANFILTER_H
#define _MEDIANFILTER_H

#include "Arduino.h"

#define filterSize 39


class MedianFilter {
  public:
    /* Global Variables */
    int filerElements[filterSize];     
    
    // constructor
    MedianFilter();
    
    // function
    int filter(int dataIn);
  
  private:
  
};


#endif
 
 

