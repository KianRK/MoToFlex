/** 
* @file ColorTable.h
* Declaration of class ColorTable.
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias J�ngel</A>
* @author <A href="mailto:martin@martin-loetzsch.de">Martin L�tzsch</A>
*/

#ifndef _ColorTable_h_
#define _ColorTable_h_

#include "Tools/Math/Vector3.h"
#include "Representations/Perception/ColorClassImage.h"
#include "Representations/Infrastructure/Image.h"

/**
* @class ColorTable
*
* @author <A href="mailto:juengel@informatik.hu-berlin.de">Matthias J�ngel</A>
* @author <A href="mailto:martin@martin-loetzsch.de">Martin L�tzsch</A>
*/
class ColorTable
{
public:
  /** Constructor */
  ColorTable(){}

  /** 
  * Constructor that creates a copy of another ColorTable.
  * \param ct The other ColorTable.
  */
  ColorTable(const ColorTable& ct){}

  /** Assignment operator
  * \param other The other ColorTable that is assigned to this one.
  * \return A reference to this object after the assignment.
  */
  ColorTable& operator=(const ColorTable& other){return *this;}

  /** 
  * Calculates the color class of a pixel.
  * @param y the y value of the pixel
  * @param u the u value of the pixel
  * @param v the v value of the pixel
  * @return the color class
  */
  virtual ColorClasses::Color getColorClass(const unsigned char y, 
    const unsigned char u, 
    const unsigned char v) const
  {
    return ColorClasses::noColor;
  }

  /**
  * Segments an image to an color class image.
  * 
  * This doesn't need to used in the image processor, but is needed for visualisation of color tables.
  * @param image A reference to the image to be segmented
  * @param colorClassImage A reference to the color class image to be created
  */
  virtual void generateColorClassImage(const Image& image, ColorClassImage& colorClassImage) const 
  {}

  /**
  * Visualizes the difference between this and another ColorTable in an Image.
  * \param colorTable The other ColorTable.
  * \param source The images used for detecting the differences.
  * \param view The image used for visulisation.
  * \param image The output image.
  */
  virtual void generateDiffImage(const ColorTable colorTable, const Image& source, const Image& view, Image& image) const
  {}


  /** Sets the color class of every 4x4x4 to noColor */
  virtual void clear() {}

  /** Sets all cubes that have the given color class to noColor */
  virtual void clearChannel(ColorClasses::Color colorClass) {}

  /** 
  * Sets the color class for a cube with the size "range" around a pixel
  * given by y,u,v to the given color class.
  */
  virtual void addColorClass(ColorClasses::Color colorClass,
                     unsigned char y, 
                     unsigned char u, 
                     unsigned char v, 
                     unsigned char range) {}

  /** 
  * Sets the color class for a pixel
  * given by y,u,v to the given color class.
  */
  virtual void addColorClass(ColorClasses::Color colorClass,
                                 unsigned char y,
                                 unsigned char u,
                                 unsigned char v) {}

  /** 
  * Replaces one color class by another for a cube with the size "range" around a pixel
  * given by y,u,v to the given color class.
  */
  virtual void replaceColorClass(ColorClasses::Color from,
                         ColorClasses::Color to,
                         unsigned char y, 
                         unsigned char u, 
                         unsigned char v, 
                         unsigned char range) {}

  /** 
  * Replaces one color class by another for a pixel
  * given by y,u,v to the given color class.
  */
  virtual void replaceColorClass(ColorClasses::Color from,
                         ColorClasses::Color to,
                         unsigned char y, 
                         unsigned char u, 
                         unsigned char v) {}

  /**
  * This function marks the outer pixels of all segmented colors in the colorspace as noColor.
  */
  virtual void shrink(){}

  /**
  * This function marks the outer pixels of the parameter color in the colorspace as noColor.
  * @param color The color to shrink
  */
  virtual void shrink(unsigned char color){}

  /**
  * This function marks all noColor pixels in the colorspace, which are neighbors of segmented 
  * pixels, with the neighboring color.
  */
  virtual void grow(){}

  /**
  * This function marks all noColor pixels in the colorspace, which are neighbors of the parameter color, 
  * with the neighboring color.
  */
  virtual void grow(unsigned char color){}

private:   
  /**
  * Returns true if the pixel y,u,v in the  colorClassesArray has more than x or x neighbors.
  * @param y the y coordinate of the pixel in the colorspace
  * @param u the u coordinate of the pixel in the colorspace
  * @param v the v coordinate of the pixel in the colorspace
  * @param x the number of desired pixels
  * @param colorClassesArray the Array with the colorClasses
  */
  bool hasXNeighbors(unsigned char y, unsigned char u, unsigned char v, int x, unsigned char colorClassesArray[64][64][64]);

  /**
  * Returns the color of one of the neighboring pixels of the pixel y,u,v.
  * If no direct neighbor is present, noColor s returned.
  * @param y the y coordinate of the pixel in the colorspace
  * @param u the u coordinate of the pixel in the colorspace
  * @param v the v coordinate of the pixel in the colorspace
  * @param colorClassesArray the Array with the colorClasses
  */
  unsigned char getNeighborColor(unsigned char y, unsigned char u, unsigned char v, unsigned char colorClassesArray[64][64][64]);
};

/**
* Streaming operator that writes a ColorTable to a stream.
* @param stream The stream to write on.
* @param ColorTable The ColorTable object.
* @return The stream.
*/ 
Out& operator<<(Out& stream, const ColorTable& ColorTable);

/**
* Streaming operator that reads a ColorTable from a stream.
* @param stream The stream from which is read.
* @param ColorTable The ColorTable object.
* @return The stream.
*/ 
In& operator>>(In& stream,ColorTable& ColorTable);

#endif   //  _ColorTable_h_
