/**
* @file ColorClasses.h
* Declaration of class ColorClasses
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/

#ifndef __ColorClasses_h__
#define __ColorClasses_h__


/**
* @class ColorClasses
* 
* Static class for color class functions.
*
* @author <A href="mailto:Tim.Laue@dfki.de">Tim Laue</A>
*/
class ColorClasses
{
public:
  enum Color
  {
    none,                   /*<! all other objects */
    noColor=none,           /*<! (same as none) all other objects */
    orange,                 /*<! ball */
    yellow,                 /*<! yellow goal */
    blue,                   /*<! blue goal */
    white,                  /*<! lines */
    green,                  /*<! field */
    black,                  /*<! most probably: nothing */
    red,                    /*<! color of red robots> */
    robotBlue,              /*<! color of blue robots> */

	  gray,                   /*<! (4legged) soft color */
	  // (MSH/ND) here new soft colors start
	  yellowOrange,           /*<! ball or yellow goal */
	  redOrange,              /*<! dark part of the ball */
	  darkGreen,              /*<! shadows on the field */
	  blackBlue,              /*<! blue jersey indistinguishable from black background */
	  blueGreen,              /*<! greenish reflections in the blue goal */
	  yellowWhite,            /*<! yellowish white */

    numOfColors             /*<! number of colors */
  };

  /**
  * Returns the name of a color class
  * @param color The color class
  * @return The name
  */
  static const char* getColorName(Color color)
  {
    switch(color) 
    {
    case none: return "none";
    case orange: return "orange"; 
    case yellow: return "yellow"; 
    case blue: return "blue";
    case green: return "green"; 
    case white: return "white";
    case black: return "black";
    case red: return "red";
    case robotBlue: return "robotBlue";
    case gray: return "gray";
    case yellowOrange: return "yellow orange";
    case redOrange: return "red orange";
    case darkGreen: return "dark green";
    case blackBlue: return "black blue";
    case blueGreen: return "blue green";
    case yellowWhite: return "yellow white";
    default: return "unknown color!"; 
    };
  }
  
  /**
  * The method returns prototypical color values for a color class for visualization.
  * @param colorClass The color class.
  * @param y The Y channel of the prototypical color.
  * @param cr The Cr channel of the prototypical color.
  * @param cb The Cb channel of the prototypical color.
  */
  static inline void getColorClassColor(ColorClasses::Color colorClass, unsigned char& y, unsigned char& cr, unsigned char& cb)
  {
    switch(colorClass)
    {
    case ColorClasses::white:
      y = 255;
      cr = 127;
      cb = 127;
      break;
    case ColorClasses::green:
      y = 180;
      cr = 0;
      cb = 0;
      break;
    case ColorClasses::orange:
      y = 164;
      cr = 255;
      cb = 0;
      break;
    case ColorClasses::yellow:
      y = 255;
      cr = 170;
      cb = 0;
      break;
    case ColorClasses::blue:
      y = 123;
      cr = 85;
      cb = 202;
      break;
    case ColorClasses::black:
      y = 32;
      cr = 128;
      cb = 128;
      break;
    case ColorClasses::red:
      y = 76;
      cr = 255;
      cb = 84;
      break;
    case ColorClasses::robotBlue:
      y = 22;
      cr = 111;
      cb = 228;
      break;

	  case ColorClasses::gray:
      //r = 127; g = 127; b = 127;
      y  = 126;
      cr = 128;
      cb = 128;
      break;
	  case ColorClasses::yellowOrange:
      //r = 217; g = 164; b = 29;
      y  = 165;
      cr = 165;
      cb = 52;
      break;
	  case ColorClasses::redOrange:
      //r = 255; g = 69; b = 0;
      y  = 116;
      cr = 226;
      cb = 62;
      break;
	  case ColorClasses::darkGreen:
      //r = 0; g = 100; b = 0;
      y  = 58;
      cr = 86;
      cb = 94;
      break;
	  case ColorClasses::blackBlue:
      //r = 0; g = 0; b = 139;
      y  = 15;
      cr = 116;
      cb = 197;
      break;
	  case ColorClasses::blueGreen:
      //r = 32; g = 178; b = 170;
      y  = 133;
      cr = 55;
      cb = 148;
      break;
	  case ColorClasses::yellowWhite:
      //r = 255; g = 255; b = 180;//b = 224;
      y  = 246;
      cr = 134;
      cb = 90;
      break;

    default:
      y = 0;
      cr = 128;
      cb = 128;
    }
  }
};

#endif //__ColorClasses_h_
