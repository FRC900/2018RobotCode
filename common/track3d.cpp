#include <iostream>
#include <limits>

#include "track3d.hpp"
#include "hungarian.hpp"

using namespace std;
using namespace cv;

// How many consecutive frames a track must be missing before
// it is erased 
const int missedFrameCountMax = 10;

/*
static Point3f screenToWorldCoords(const Rect &screen_position, double avg_depth, const Point2f &fov_size, const Size &frame_size, float cameraElevation)
{
	TODO COMMENT THIS
	Method:
		find the center of the rect
		compute the distance from the center of the rect to center of image (pixels)
		convert to degrees based on fov and image size
		do a polar to cartesian cordinate conversion to find x,y,z of object
	Equations:
		x=rsin(inclination) * cos(azimuth)
		y=rsin(inclination) * sin(azimuth)
		z=rcos(inclination)
	Notes:
		Z is up, X is left-right, and Y is forward
		(0,0,0) = (r,0,0) = right in front of you
	

	Point2f rect_center(
			screen_position.tl().x + (screen_position.width  / 2.0),
			screen_position.tl().y + (screen_position.height / 2.0));
	Point2f dist_to_center(
			rect_center.x - (frame_size.width / 2.0),
			-rect_center.y + (frame_size.height / 2.0));

	

// This uses formula from http://www.chiefdelphi.com/forums/showpost.php?p=1571187&postcount=4
	float azimuth = atan(dist_to_center.x / (.5 * frame_size.width / tan(fov_size.x / 2)));
	float inclination = atan(dist_to_center.y / (.5 * frame_size.height / tan(fov_size.y / 2))) - cameraElevation;

	Point3f retPt(
			avg_depth * cosf(inclination) * sinf(azimuth),
			avg_depth * cosf(inclination) * cosf(azimuth),
			avg_depth * sinf(inclination));

	//cout << "Distance to center: " << dist_to_center << endl;
	//cout << "Actual Inclination: " << inclination << endl;
	//cout << "Actual Azimuth: " << azimuth << endl;
	//cout << "Actual location: " << retPt << endl;

	return retPt;
}

static Rect worldToScreenCoords(const Point3f &_position, const ObjectType &_type, const Point2f &fov_size, const Size &frame_size, float cameraElevation)
{
	// TODO : replace magic numbers with an object depth property
	// This constant is half a ball diameter (9.75-ish inches), converted to meters
	// For example, goals will have 0 depth since we're just shooting at
	// a plane. 3d objects will have depth, though, so we track the center of the
	// rather than the front.
	float r = sqrtf(_position.x * _position.x + _position.y * _position.y + _position.z * _position.z); // - (4.572 * 25.4)/1000.0;
	float azimuth = asinf(_position.x / sqrt(_position.x * _position.x + _position.y * _position.y));
	float inclination = asinf( _position.z / r ) + cameraElevation;

	//inverse of formula in screenToWorldCoords()
	Point2f dist_to_center(
			tan(azimuth) * (0.5 * frame_size.width / tan(fov_size.x / 2)),
			tan(inclination) * (0.5 * frame_size.height / tan(fov_size.y / 2)));
	
	//cout << "Distance to center: " << dist_to_center << endl;
	Point2f rect_center(
			dist_to_center.x + (frame_size.width / 2.0),
			-dist_to_center.y + (frame_size.height / 2.0));

	Point2f angular_size( 2.0 * atan2f(_type.width(), (2.0*r)), 2.0 * atan2f(_type.height(), (2.0*r)));
	Point2f screen_size(
			angular_size.x * (frame_size.width / fov_size.x),
			angular_size.y * (frame_size.height / fov_size.y));

	Point topLeft(
			cvRound(rect_center.x - (screen_size.x / 2.0)),
			cvRound(rect_center.y - (screen_size.y / 2.0)));
			return Rect(topLeft.x, topLeft.y, cvRound(screen_size.x), cvRound(screen_size.y));
}
*/
TrackedObject::TrackedObject(int               id,
							 const ObjectType &type_in,
							 const Rect       &screen_position,
							 double            avg_depth,
							 const Point2f    &fov_size,
							 const Size       &frame_size,
							 float             camera_elevation,
							 float             dt,
							 float             accel_noise_mag,
							 size_t            historyLength) :
		type_(type_in),
		detectHistory_(historyLength),
		positionHistory_(historyLength),
		KF_(type_.screenToWorldCoords(screen_position, avg_depth, fov_size, frame_size, camera_elevation),
			dt, accel_noise_mag),
		missedFrameCount_(0),
		cameraElevation_(camera_elevation)
{
	setPosition(screen_position, avg_depth, fov_size, frame_size);
	setDetected();

	// Label with base-26 letter ID (A, B, C .. Z, AA, AB, AC, etc)
	do
	{
		id_ += (char)(id % 26 + 'A');
		id /= 26;
	}
	while (id != 0);
	reverse(id_.begin(), id_.end());
}


#if 0
TrackedObject::~TrackedObject()
{
}
#endif

// Set the position based on x,y,z coords

void TrackedObject::setPosition(const Point3f &new_position)
{
	position_ = new_position;
	addToPositionHistory(position_);
}

// Set the position based on a rect on the screen and depth info from the zed
void TrackedObject::setPosition(const Rect &screen_position, double avg_depth,
		                        const Point2f &fov_size, const Size &frame_size)
{
	setPosition(type_.screenToWorldCoords(screen_position, avg_depth, fov_size, frame_size, cameraElevation_));
}

#if 0
void TrackedObject::adjustPosition(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	//Eigen::AngleAxisd rot(0.5*M_PI, Eigen::Vector3d::UnitZ());

	Eigen::Vector3d old_pos_vec(_position.x, _position.y, _position.z);
	Eigen::Vector3d new_pos_vec = delta_robot * old_pos_vec;

	position_ = Point3f(new_pos_vec[0], new_pos_vec[1], new_pos_vec[2]);

	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		Eigen::Vector3d old_pos_vector(it->x, it->y, it->z);
		Eigen::Vector3d new_pos_vector = delta_robot * old_pos_vector;
		*it = Point3f(new_pos_vector[0], new_pos_vector[1], new_pos_vector[2]);
	}
}
#endif
void TrackedObject::adjustPosition(const Mat &transform_mat, float depth, const Point2f &fov_size, const Size &frame_size)
{
	//get the position of the object on the screen
	Rect screen_rect = getScreenPosition(fov_size,frame_size);
	Point screen_pos(screen_rect.tl().x + screen_rect.width / 2, screen_rect.tl().y + screen_rect.height / 2);

	//create a matrix to hold positon for matrix multiplication
	Mat pos_mat(3,1,CV_64FC1);
	pos_mat.at<double>(0,0) = screen_pos.x;
	pos_mat.at<double>(0,1) = screen_pos.y;
	pos_mat.at<double>(0,2) = 1.0;

	//correct the position
	Mat new_screen_pos_mat(3,1,CV_64FC1);
	new_screen_pos_mat = transform_mat * pos_mat;
	Point new_screen_pos(new_screen_pos_mat.at<double>(0),new_screen_pos_mat.at<double>(1));

	//create a dummy bounding rect because setPosition requires a bounding rect as an input rather than a point
	Rect new_screen_rect(new_screen_pos.x,new_screen_pos.y,0,0);
	setPosition(new_screen_rect,depth,fov_size,frame_size);
	//update the history
	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		screen_rect = type_.worldToScreenCoords(*it,fov_size,frame_size, cameraElevation_);
		screen_pos = Point(screen_rect.tl().x + screen_rect.width / 2, screen_rect.tl().y + screen_rect.height / 2);
		pos_mat.at<double>(0,0) = screen_pos.x;
		pos_mat.at<double>(0,1) = screen_pos.y;
		pos_mat.at<double>(0,2) = 1.0;
		Mat new_screen_pos_mat = transform_mat * pos_mat;
		Point new_screen_pos(new_screen_pos_mat.at<double>(0),new_screen_pos_mat.at<double>(1));
		Rect new_screen_rect(new_screen_pos.x,new_screen_pos.y,0,0);
		*it = type_.screenToWorldCoords(new_screen_rect, depth, fov_size, frame_size, cameraElevation_);
	}
}

// Mark the object as detected in this frame
void TrackedObject::setDetected(void)
{
	detectHistory_.push_back(true);
	missedFrameCount_ = 0;
}

// Clear the object detect flag for this frame.
// Probably should only happen when moving to a new
// frame, but may be useful in other cases
void TrackedObject::clearDetected(void)
{
	detectHistory_.push_back(false);
	missedFrameCount_ += 1;
}

bool TrackedObject::tooManyMissedFrames(void) const
{
	// Hard limit on the number of consecutive missed
	// frames before dropping a track
	if (missedFrameCount_ > missedFrameCountMax)
		return true;

	// Be more aggressive about dropping tracks which
	// haven't been around long - kill them off if 
	// they are seen in less than 33% of frames
	if (detectHistory_.size() <= 10)
	{
		size_t detectCount = 0;
		for (auto it = detectHistory_.begin();  it != detectHistory_.end(); ++it)
			if (*it)
				detectCount += 1;
		if (((double)detectCount / detectHistory_.size()) <= 0.34)
			return true;
	}
	return false;
}

// Keep a history of the most recent positions
// of the object in question
void TrackedObject::addToPositionHistory(const Point3f &pt)
{
	positionHistory_.push_back(pt);
}

// Return a vector of points of the position history
// of the object (hopefully) relative to current screen location
vector <Point> TrackedObject::getScreenPositionHistory(const Point2f &fov_size, const Size &frame_size) const
{
	vector <Point> ret;

	for (auto it = positionHistory_.begin(); it != positionHistory_.end(); ++it)
	{
		Rect screen_rect(type_.worldToScreenCoords(*it,fov_size,frame_size, cameraElevation_));
		ret.push_back(Point(cvRound(screen_rect.x + screen_rect.width / 2.),cvRound( screen_rect.y + screen_rect.height / 2.)));
	}
	return ret;
}

const double minDisplayRatio = 0.3;

// Return the percent of last detectHistory_.capacity() frames
// the object was seen
double TrackedObject::getDetectedRatio(void) const
{
	// Need at least 2 frames to believe there's something real
	if (detectHistory_.size() <= 1)
		return 0.01;

	// Don't display stuff which hasn't been detected recently.
	if (missedFrameCount_ >= 3)
		return 0.01;

	size_t detectCount = 0;
	for (auto it = detectHistory_.begin();  it != detectHistory_.end(); ++it)
		if (*it)
			detectCount += 1;

	// For newly added tracks make sure only 1 frame is missed at most
	// while the first quarter of the buffer is filled and at most
	// two are missed while filling up to half the size of the buffer
	if (detectHistory_.size() < (detectHistory_.capacity()/2))
	{
		if (detectCount < (detectHistory_.size() - 2))
			return 0.01;
		if ((detectHistory_.size() <= (detectHistory_.capacity()/4)) && (detectCount < (detectHistory_.size() - 1)))
			return 0.01;

		// Ramp up from minDisplayRatio so that at 10 hits it will
		// end up at endRatio = 10/20 = 50% or 9/20 = 45%
		// 2:2 = 32.5%
		// 3:3 = 35%
		// 4:4 = 37.5%
		// 5:5 = 40%
		// 6:6 = 42.5%
		// 7:7 = 45%
		// 8:8 = 47.5%
		// 9:9 = 50%
		double endRatio =  (detectHistory_.capacity() / 2.0 - (detectHistory_.size() - detectCount)) / detectHistory_.capacity();
		return minDisplayRatio + (detectHistory_.size() - 2.0) * (endRatio - minDisplayRatio) / (detectHistory_.capacity() / 2.0 - 2.0);
	}
	double detectRatio = (double)detectCount / detectHistory_.capacity();
	return detectRatio;
}


Rect TrackedObject::getScreenPosition(const Point2f &fov_size, const Size &frame_size) const
{
	return type_.worldToScreenCoords(position_, fov_size, frame_size, cameraElevation_);
}


//fit the contour of the object into the rect of it and return the area of that
//kinda gimmicky but pretty cool and might have uses in the future
double TrackedObject::contourArea(const Point2f &fov_size, const Size &frame_size) const
{
	Rect screen_position = getScreenPosition(fov_size, frame_size);
	float scale_factor_x = (float)screen_position.width / type_.width();
	float scale_factor_y = (float)screen_position.height / type_.height();
	float scale_factor   = min(scale_factor_x, scale_factor_y);

	vector<Point2f> scaled_contour;
	for(size_t i = 0; i < type_.shape().size(); i++)
	{
		scaled_contour.push_back(type_.shape()[i] * scale_factor);
	}

	return cv::contourArea(scaled_contour);
}

Point3f TrackedObject::predictKF(void)
{
	return KF_.GetPrediction();
}


Point3f TrackedObject::updateKF(Point3f pt)
{
	return KF_.Update(pt);
}

#if 0
void TrackedObject::adjustKF(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	KF_.adjustPrediction(delta_robot);
}
#endif
void TrackedObject::adjustKF(Point3f delta_pos)
{
	KF_.adjustPrediction(delta_pos);
}


//Create a tracked object list
// those stay constant for the entire length of the run
TrackedObjectList::TrackedObjectList(const Size &imageSize, const Point2f &fovSize, float cameraElevation) :
	detectCount_(0),
	imageSize_(imageSize),
	fovSize_(fovSize),
	cameraElevation_(cameraElevation)
{
}
#if 0
// Adjust position for camera motion between frames using fovis
void TrackedObjectList::adjustLocation(const Eigen::Transform<double, 3, Eigen::Isometry> &delta_robot)
{
	for (auto it = list_.begin(); it != list_.end(); ++it)
	{
		it->adjustPosition(delta_robot);
		it->adjustKF(delta_robot);
	}
}
#endif
// Adjust position for camera motion between frames using optical flow
void TrackedObjectList::adjustLocation(const Mat &transform_mat)
{
	for (auto it = list_.begin(); it != list_.end(); ++it)
	{
		//measure the amount that the position changed and apply the same change to the kalman filter
		Point3f old_pos = it->getPosition();
		//compute r and use it for depth (assume depth doesn't change)
		float r = sqrt(it->getPosition().x * it->getPosition().x + it->getPosition().y * it->getPosition().y + it->getPosition().z * it->getPosition().z);
		it->adjustPosition(transform_mat, r, fovSize_, imageSize_);
		Point3f delta_pos = it->getPosition() - old_pos;

		it->adjustKF(delta_pos);
	}
}

// Get position history for each tracked object
vector<vector<Point>> TrackedObjectList::getScreenPositionHistories(void) const
{
	vector<vector<Point>> ret;
	for (auto it = list_.begin(); it != list_.end(); ++it)
		ret.push_back(it->getScreenPositionHistory(fovSize_, imageSize_));
	return ret;
}

// Simple printout of list into stdout
void TrackedObjectList::print(void) const
{
	for (auto it = list_.cbegin(); it != list_.cend(); ++it)
	{
		cout << it->getId() << " location ";
		Point3f position = it->getPosition();
		cout << "(" << position.x << "," << position.y << "," << position.z << ")" << endl;
	}
}

// Return list of detect info for external processing
void TrackedObjectList::getDisplay(vector<TrackedObjectDisplay> &displayList) const
{
	displayList.clear();
	TrackedObjectDisplay tod;
	for (auto it = list_.cbegin(); it != list_.cend(); ++it)
	{
		tod.id       = it->getId();
		tod.rect     = it->getScreenPosition(fovSize_, imageSize_);
		tod.ratio    = it->getDetectedRatio();
		tod.position = it->getPosition();
		tod.name	 = it->getType().name();
		displayList.push_back(tod);
	}
}

const double dist_thresh_ = 1.0; // FIX ME!
//#define VERBOSE_TRACK


// Process a set of detected rectangles
// Each will either match a previously detected object or
// if not, be added as new object to the list
void TrackedObjectList::processDetect(const vector<Rect> &detectedRects,
									  const vector<float> &depths,
									  const vector<ObjectType> &types)
{
	vector<Point3f> detectedPositions;
#ifdef VERBOSE_TRACK
	if (detectedRects.size() || list_.size())
		cout << "---------- Start of process detect --------------" << endl;
	print();
	if (detectedRects.size() > 0)
		cout << detectedRects.size() << " detected objects" << endl;
#endif
	for (size_t i = 0; i < detectedRects.size(); i++)
	{
		detectedPositions.push_back(
				types[i].screenToWorldCoords(detectedRects[i], depths[i], fovSize_, imageSize_, cameraElevation_));
#ifdef VERBOSE_TRACK
		cout << "Detected rect [" << i << "] = " << detectedRects[i] << " positions[" << detectedPositions.size() - 1 << "]:" << detectedPositions[detectedPositions.size()-1] << endl;
#endif
	}
	// TODO :: Combine overlapping detections into one?

	// Maps tracks to the closest new detected object.
	// assignment[track] = index of closest detection
	vector<int> assignment;
	if (list_.size())
	{
		size_t tracks = list_.size();		          // number of tracked objects from prev frames
		size_t detections = detectedPositions.size(); // number of detections this frame

		//Cost[t][d] is the distance between old tracked location t
		//and newly detected object d's position 
		vector< vector<double> > Cost(tracks,vector<double>(detections));

		// Calculate cost for each track->pair combo
		// The cost here is just the distance between them
		// Also check to see if the types are the same, if they are not then set the cost extremely high so that it's never matched
		auto it = list_.cbegin();
		for(size_t t = 0; t < tracks;  ++t, ++it)
		{
			// Point3f prediction=tracks[t]->prediction;
			// cout << prediction << endl;
			for(size_t d = 0; d < detections; d++)
			{
				const ObjectType it_type = it->getType();
				if(types[d] == it_type) {
					Point3f diff = it->getPosition() - detectedPositions[d];
					Cost[t][d] = sqrtf(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
				} else {
					Cost[t][d] = numeric_limits<double>::max();
				}
			}
		}

		// Solving assignment problem (find minimum-cost assignment
		// between tracks and previously-predicted positions)
		AssignmentProblemSolver APS;
		APS.Solve(Cost, assignment, AssignmentProblemSolver::optimal);

#ifdef VERBOSE_TRACK
		// assignment[i] holds the index of the detection assigned
		// to track i.  assignment[i] is -1 if no detection was
		// matchedto that particular track
		cout << "After APS : "<<endl;
		for(size_t i = 0; i < assignment.size(); i++)
			cout << i << ":" << assignment[i] << endl;
#endif
		// clear assignment from pairs with large distance
		for(size_t i = 0; i < assignment.size(); i++)
			if ((assignment[i] != -1) && (Cost[i][assignment[i]] > dist_thresh_))
				assignment[i] = -1;
	}

	// Search for unassigned detects and start new tracks for them.
	// This will also handle the case where no tracks are present,
	// since assignment will be empty in that case - everything gets added
	for(size_t i = 0; i < detectedPositions.size(); i++)
	{
		if (find(assignment.begin(), assignment.end(), i) == assignment.end())
		{
#ifdef VERBOSE_TRACK
			cout << "New assignment created " << i << endl;
#endif
			list_.push_back(TrackedObject(detectCount_++, types[i], detectedRects[i], depths[i], fovSize_, imageSize_, cameraElevation_));

#ifdef VERBOSE_TRACK
			cout << "New assignment finished" << endl;
#endif
		}
	}

	auto tr = list_.begin();
	auto as = assignment.begin();
	while ((tr != list_.end()) && (as != assignment.end()))
	{
		// If track updated less than one time, than filter state is not correct.
#ifdef VERBOSE_TRACK
		cout << "Predict: " << endl;
#endif
		Point3f prediction = tr->predictKF();
#ifdef VERBOSE_TRACK
		cout << "prediction:" << prediction << endl;
#endif

		if(*as != -1) // If we have assigned detect, then update using its coordinates
		{
#ifdef VERBOSE_TRACK
			cout << "Update match: " << endl;
#endif
			tr->setPosition(tr->updateKF(detectedPositions[*as]));
#ifdef VERBOSE_TRACK
			cout << tr->getScreenPosition(fovSize_, imageSize_) << endl;
#endif
			tr->setDetected();
		}
		else          // if not continue using predictions
		{
#ifdef VERBOSE_TRACK
			cout << "Update no match: " << endl;
#endif
			tr->setPosition(tr->updateKF(prediction));
#ifdef VERBOSE_TRACK
			cout << tr->getScreenPosition(fovSize_, imageSize_) << endl;
#endif
			tr->clearDetected();
		}

		++tr;
		++as;
	}

	// Remove tracks which haven't been seen in a while
	for (auto it = list_.begin(); it != list_.end(); )
	{
		if (it->tooManyMissedFrames()) // For now just remove ones for
		{                              // which detectList is empty
#ifdef VERBOSE_TRACK
			cout << "Dropping " << it->getId() << endl;
#endif
			it = list_.erase(it);
		}
		else
		{
			++it;
		}
	}
#ifdef VERBOSE_TRACK
	print();
	if (detectedRects.size() || list_.size())
		cout << "---------- End of process detect --------------" << endl;
#endif
}
