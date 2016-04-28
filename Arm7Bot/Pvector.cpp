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

#include "PVector.h"

PVector::PVector() {
  x = y = z = 0.0;
}

PVector::PVector(double _x, double _y, double _z) {
  x = _x; y = _y; z = _z;
}

void PVector::add(PVector p) {
  x += p.x;
  y += p.y;
  z += p.z;
}

void PVector::normalize() {
  double l = sqrt(x * x + y * y + z * z);
  x /= l;
  y /= l;
  z /= l;
}

double PVector::dot(PVector p) {
  return x * p.x + y * p.y + z * p.z;
}

double PVector::dist(PVector p) {
  double dist_x = x - p.x;
  double dist_y = y - p.y;
  double dist_z = z - p.z;
  return sqrt(dist_x * dist_x + dist_y * dist_y + dist_z * dist_z);
}

