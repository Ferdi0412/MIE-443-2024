#pragma once

#include <opencv2/highgui/highgui.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <string>

/**
 * Boxes is a class that sets up the "config" of a test, defining the locations (coords) of boxes and the templates
 * used by the image processing of the boxes.
 *
 * @property templates -> A vector of <cv::Mat> defining the templates
 * @property coords    -> A vector of <std::vector<float>> defining the x-, y- and phi- coordinates of each box
 *
 * @method load_coords
 * @method load_templates
*/
class Boxes {
	public:
		std::vector<cv::Mat>            templates;
		std::vector<std::vector<float>> coords;
		std::vector<std::string>        template_names;


	public:
		/**
		 * load_coords loads the coordinates of the targets as defined in gazebo_coords.xml
		*/
		bool load_coords();

		/**
		 * load_coords loads the coordinates of the targets as defined in an .xml file in the ../boxes_database directory
		 * @param file_name <std::string> the name of the file in the ../boxes_database directory
		*/
		bool load_coords( std::string file_name );

		/**
		 * load_templates loads the templates for images to be recognized as defined in templates.xml
		*/
		bool load_templates();

		/**
		 * load_templates loads the templates as defined in a .xml file in the ../boxes_database directory
		 * @param file_name <std::string> the name of the file in the ../boxes_database directory
		*/
		bool load_templates( std::string file_name );

		/**
		 * get_template_filename - returns the name of the file used in identifying the template OR "UNIDENTIFIED" OR "Blank Image"
		*/
		std::string get_template_filename( int template_id );
};
