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

#ifndef _FORCEFILTER_H
#define _FORCEFILTER_H

#include "Arduino.h"

#define forceFilterSize 30


class ForceFilter {
  public:
    /* Global Variables */
    int filerElements[forceFilterSize];     
    
    // constructor
    ForceFilter();
    
    // function
    int filter(int dataIn);
  
  private:
  
};


#endif
 
 

