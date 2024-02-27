# Contest 2
Here we are building files to traverse an unknown environtment, to move and image targets at known locations.

## Contest files
The following files were provided with the contest package:
1. boxes
2. imagePipeline
3. navigation
4. robot_pose
5. contest2.cpp
6. webcam_publisher.cpp

## Header files
Add these to the [include](include) directory.

## Function files
Add these to the [src](src) directory.

## Testing
To test, modify the [src/priv-test.cpp](src/priv-test.cpp) file. This will **NOT** be commited to Git, and so you have free range to do with it as you like.

```shell
rosrun mie443_contest2 priv_test
```

## Gazebo
To open gazerbo, run the following command:

```shell
roslaunch mie443_contest2 turtlebot_world.launch world:=1
```

# OpenCV Feature Matching

## Steps

### 1. Detect Features
Here we detect features of the image input (and of the templates to compare it with). This step provides 2 components:

1. The **position** of the key features.
2. A **descriptor** of the key features.

<br>An example of how this is done is as follows:

```C++
// Assume this is given
cv::Mat img_to_match;

// Create an instance of the orb class (using a cv::Ptr -> OpenCV convention)
cv::Ptr<cv::ORB> orb_feature_detector = cv::ORB::create();

// You need the following parameters to store the results
cv::Mat                   img_feature_descriptors;
std::vector<cv::KeyPoint> img_feature_positions;

// // Run the detectAndCompute method (NOTE: using a '->' instead of '.' as we have a pointer to the object)
orb_feature_detector->detectAndCompute(
    makeGrayscale(img),
    cv::Mat(),
    img_feature_positions,
    img_feature_descriptors
);
```

<br>There are a couple options for the feature_detector model. The following are promising alternatives to explore:

1. SURF  `cv::xfeatures2d::SURF`
2. ORB   `cv::ORB`
3. AKAZE `cv::AKAZE`

<br>There are some more alternatives, but these appear to be some of those that should best handle translated/rotated features when matching in later steps.

<br>You may have noticed that **makeGrayscale** is used often. This is not an actual function from OpenCV, and is not strictly necessary, but it appears to be best practice, according to online doccumentation.

```C++
// Function to return a grayscale copy of the input image
cv::Mat makeGrayscale( const cv::Mat& input_img ) {
    cv::Mat new_img;

    // If only 1 channel -> already a grayscale image -> return copy of input as-is
    if ( img.channels() == 1 )
        return input_img.clone();

    // If 3 channels -> presumably BGR (typically used over RGB in OpenCV it appears) -> create a grayscale copy of it
    else if ( img.channels() == 3 )
        cv::cvtColor( input_img, new_img, cv::COLOR_BGR2GRAY );

    // If 4 channels -> presumably BGRA -> create grayscale copy of it
    else if ( img.channels() == 4 )
        cv::cvtColor( input_img, new_imag, cv::COLOR_BGRA2GRAY );

    // If you find other useful cases, feel free to expand it...
    // Otherwise find some way to handle base case...
    // Maybe return an empty grayscale image?
    // Maybe throw an exception?
    else
        throw std::exception("Could not find a useful conversion of colour image to grayscale!");

    return new_img;
}
```

### 2. Feature Matching
Here we compare and match the features between the template image and the input image. This assumes that the features have been detected for both images. Here is an example of how to do this:

```C++
// If using LOWE filter, assign some threshold
#define LOWE_THRESHOLD 0.7

// Assume the following is given
cv::Mat template_desc;
cv::Mat img_desc;

// Create an instance of the flannBasedMatcher class (using a cv::Ptr -> OpenCV convention)
cv::Ptr<cv::FlannBasedMatcher> flann_feature_matcher = cv::FlannBasedMatcher::create();

// You need the following parameter(s) to store the results
cv::vector<cv::DMatch> filtered_matches;
cv::vector<cv::vector<cv::DMatch>> unfiltered_matches;

// Option 1: Use the matcher's match method
flann_feature_matcher->match( template_desc, img_desc, filtered_matches );

// Option 2: Use the k-nearest-neighbour match, and apply a Lowe filter
//           For more details see: https://docs.opencv.org/3.4/d5/d6f/tutorial_feature_flann_matcher.html
flann_feature_matcher->match( template_desc, img_desc, unfiltered_matches );

for ( size_t i = 0; i < unfiltered_matches.size(); i++ ) {
    if ( unfiltered_matches[i][0].distance < LOWE_THRESHOLD * unfiltered_matches[i][1].distance )
        filtered_matches.push_back(unfiltered_matches[i][0]);
}
```

<br>Above we see something called the **Lowe filter**. This is a method referenced in a lot of documentation online on feature matching. Worth exploring.

<br>Again, we have some options for the matcher object:

1. Flann `cv::FlannBasedMatcher`
2. Brute-Force `cv::BFMatcher`

### 3. Checking number of matched features
The best approach to comparing *"goodness"* of match between images using feature matching, is to check the number of matched features (I cannot find a "goodness" of feature-match).

```C++
// Have a number of features to match as a threshold
#define NUMBER_TO_MATCH 45

// Check the number of features matched
if ( filtered_matches.size > NUMBER_TO_MATCH )
    std::cout << "You have a match!";

else
    std::cout << "No dice :(";
```

### 4. Displaying matches
You can either display the features of a given image:

```C++
// You need a new variable to store the output image
cv::Mat image_to_display;

// Annotate the features found
cv::drawKeypoints(
    makeGrayscale(input_img),
    input_feature_positions,
    image_to_display
);

// Display the image
cv::imshow("Key-features", image_to_display);
cv::waitKey(10);
```

<br>Or you can display the matched features between images:

```C++
// You need a new variable to store the output image
cv::Mat image_to_display;

// Annotate the matched features
cv::drawMatches(
    makeGrayscale(template_img),
    template_feature_positions,
    makeGrayscale(input_img),
    input_feature_positions,
    filtered_matches,
    image_to_display
);

// Display the images
cv::imshow("Matches", image_to_display);
cv::waitKey(10);
```
