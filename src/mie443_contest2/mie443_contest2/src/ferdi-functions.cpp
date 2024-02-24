#ifndef FERDI_FUNCTIONS_CPP
#define FERDI_FUNCTIONS_CPP

#include <imagePipeline.h>

namespace Ferdi {
    static cv::Mat latest_res, latest_locations;

    /**
     * Ferdi::match_templates tries to match img to template_img
     *
     * @implements the cv::matchTemplate; cv::findNonZero methods to search for template_img matches
     * @note       Ferdi::latest_res; Ferdi::latest_locations [both PRIVATE] to store search results
     *
     * @param img          the image to match
     * @param template_img the template to match
     * @param match_method the method to use in matching
     * @param threshold    the threshold to use in matching
     *
     * @returns a count of the matches
    */
    unsigned int match_templates( const cv::Mat& img, const cv::Mat& template_img, int match_method, double threshold ) {
        cv::matchTemplate( img, template_img, latest_res, match_method );
        cv::findNonZero( latest_res > threshold, latest_locations );
        return latest_locations.total();
    }


    /**
     * Ferdi::search_templates tries to match img to template_img
     *
     * @param img the image to match
     * @param template_img the template to match
     *
     * @a Uses the <TM_CCOEFF_NORMED> match method
     * @a Uses a threshold value of <0.8> to match
    */
    bool search_templates( const cv::Mat& img, const cv::Mat& template_img ) {
        return match_templates( img, template_img ) > 0;
    }

    /**
     * Ferdi::draw_templates tries to draw rectangles around each detected instance of template_img
     *
     * @note  Assumes that search_templates has already been run on it...
     *
     * @param img the image to match
     * @param template_img the template to match
    */
    cv::Mat draw_templates( const cv::Mat& img, const cv::Mat& template_img) {
        cv::Mat drawing = img.clone();
        for ( cv::Point p : latest_locations )
            cv::rectangle( drawing, p, cv::Point(p.x + template_img.cols, p.y + template_img.rows), Scalar(0, 0, 255), 4 );
        return drawing;
    }
}

#endif
