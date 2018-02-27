// A class for defining objects we're trying to 
// detect.  The class stores information about shape 
// and size of the objects in real-world measurements
#include "objtype.hpp"

using namespace std;
using namespace cv;

ObjectType::ObjectType(int contour_type_id=1) {
	switch(contour_type_id) {
		//loads one of the preset shapes into the
		//object

		case 1: //a ball!
			depth_ = 0.2476; // meters
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0, depth_));
			contour_.push_back(Point2f(depth_, depth_));
			contour_.push_back(Point2f(depth_,0));
			name_="ball";
			break;

		case 2: //a bin (just because)
			depth_ = 0.5588;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.5842));
			contour_.push_back(Point2f(0.5842,0.5842));
			contour_.push_back(Point2f(0.5842,0));
			name_="bin";
			break;

		case 3: //2016 Goal
			{
				depth_ = 0;
				float max_y = .3048;
				contour_.push_back(Point2f(0, max_y - 0));
				contour_.push_back(Point2f(0, max_y - 0.3048));
				contour_.push_back(Point2f(0.0508, max_y - 0.3048));
				contour_.push_back(Point2f(0.0508, max_y - 0.0508));
				contour_.push_back(Point2f(0.508-0.0508, max_y - 0.0508));
				contour_.push_back(Point2f(0.508-0.0508, max_y - 0.3048));
				contour_.push_back(Point2f(0.508, max_y - 0.3048));
				contour_.push_back(Point2f(0.508, max_y - 0));
				name_="goal";
			}
			break;
		case 4: //top piece of tape (2017)
			depth_ = 0;
			real_height_ = 1.9812;  //76 inches + 4 * 1/2 height
			real_height_ -= .22225; // 8.75 in camera height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0, 0.1010));
			contour_.push_back(Point2f(0,0.118954963068));
			contour_.push_back(Point2f(0.01272380885,0.1230581348));
			contour_.push_back(Point2f(0.0255358792974,0.126876796969));
			contour_.push_back(Point2f(0.0384298504578,0.130409053701));
			contour_.push_back(Point2f(0.0513993207851,0.133653151317));
			contour_.push_back(Point2f(0.0644378512496,0.136607479199));
			contour_.push_back(Point2f(0.0775389685348,0.139270570596));
			contour_.push_back(Point2f(0.0906961682517,0.141641103346));
			contour_.push_back(Point2f(0.103902918167,0.143717900537));
			contour_.push_back(Point2f(0.117152661449,0.145499931089));
			contour_.push_back(Point2f(0.130438819917,0.146986310268));
			contour_.push_back(Point2f(0.143754797315,0.148176300121));
			contour_.push_back(Point2f(0.15709398258,0.149069309847));
			contour_.push_back(Point2f(0.170449753128,0.149664896088));
			contour_.push_back(Point2f(0.183815478141,0.149962763151));
			contour_.push_back(Point2f(0.197184521859,0.149962763151));
			contour_.push_back(Point2f(0.210550246872,0.149664896088));
			contour_.push_back(Point2f(0.22390601742,0.149069309847));
			contour_.push_back(Point2f(0.237245202685,0.148176300121));
			contour_.push_back(Point2f(0.250561180083,0.146986310268));
			contour_.push_back(Point2f(0.263847338551,0.145499931089));
			contour_.push_back(Point2f(0.277097081833,0.143717900537));
			contour_.push_back(Point2f(0.290303831748,0.141641103346));
			contour_.push_back(Point2f(0.303461031465,0.139270570596));
			contour_.push_back(Point2f(0.31656214875,0.136607479199));
			contour_.push_back(Point2f(0.329600679215,0.133653151317));
			contour_.push_back(Point2f(0.342570149542,0.130409053701));
			contour_.push_back(Point2f(0.355464120703,0.126876796969));
			contour_.push_back(Point2f(0.36827619115,0.1230581348));
			contour_.push_back(Point2f(0.381,0.118954963068));	
			contour_.push_back(Point2f(0.381, 0));
			contour_.push_back(Point2f(0.381,0.0122875187586));
			contour_.push_back(Point2f(0.368470376004,0.0172542619015));
			contour_.push_back(Point2f(0.355811431603,0.0218814793796));
			contour_.push_back(Point2f(0.343032365293,0.0261658088713));
			contour_.push_back(Point2f(0.330142462857,0.0301041372113));
			contour_.push_back(Point2f(0.317151090612,0.0336936026522));
			contour_.push_back(Point2f(0.304067688612,0.0369315969449));
			contour_.push_back(Point2f(0.29090176378,0.0398157672328));
			contour_.push_back(Point2f(0.277662883004,0.0423440177624));
			contour_.push_back(Point2f(0.264360666186,0.0445145114055));
			contour_.push_back(Point2f(0.251004779249,0.0463256709944));
			contour_.push_back(Point2f(0.237604927115,0.0477761804682));
			contour_.push_back(Point2f(0.224170846654,0.0488649858284));
			contour_.push_back(Point2f(0.210712299607,0.0495912959055));
			contour_.push_back(Point2f(0.197239065494,0.0499545829336));
			contour_.push_back(Point2f(0.183760934506,0.0499545829336));
			contour_.push_back(Point2f(0.170287700393,0.0495912959055));
			contour_.push_back(Point2f(0.156829153346,0.0488649858284));
			contour_.push_back(Point2f(0.143395072885,0.0477761804682));
			contour_.push_back(Point2f(0.129995220751,0.0463256709944));
			contour_.push_back(Point2f(0.116639333814,0.0445145114055));
			contour_.push_back(Point2f(0.103337116996,0.0423440177624));
			contour_.push_back(Point2f(0.09009823622,0.0398157672328));
			contour_.push_back(Point2f(0.0769323113878,0.0369315969449));
			contour_.push_back(Point2f(0.0638489093876,0.0336936026522));
			contour_.push_back(Point2f(0.0508575371435,0.0301041372113));
			contour_.push_back(Point2f(0.0379676347069,0.0261658088713));
			contour_.push_back(Point2f(0.0251885683974,0.0218814793796));
			contour_.push_back(Point2f(0.0125296239964,0.0172542619015));
			contour_.push_back(Point2f(0,0.0122875187586));
			name_ = "top_boiler_tape";
			break;	
		case 5: //bottom piece of tape (2017)
			depth_ = 0;
			real_height_ = 1.7272; //5ft 7inches + 1/2 * 2in height
			real_height_ -= .22225; // 8.75 in camera height
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.0489549630683));
			contour_.push_back(Point2f(0.01272380885,0.0530581347998));
			contour_.push_back(Point2f(0.0255358792974,0.0568767969687));
			contour_.push_back(Point2f(0.0384298504578,0.0604090537011));
			contour_.push_back(Point2f(0.0513993207851,0.0636531513167));
			contour_.push_back(Point2f(0.0644378512496,0.0666074791992));
			contour_.push_back(Point2f(0.0775389685348,0.0692705705958));
			contour_.push_back(Point2f(0.0906961682517,0.0716411033459));
			contour_.push_back(Point2f(0.103902918167,0.073717900537));
			contour_.push_back(Point2f(0.117152661449,0.0754999310894));
			contour_.push_back(Point2f(0.130438819917,0.0769863102677));
			contour_.push_back(Point2f(0.143754797315,0.0781763001206));
			contour_.push_back(Point2f(0.15709398258,0.0790693098467));
			contour_.push_back(Point2f(0.170449753128,0.0796648960881));
			contour_.push_back(Point2f(0.183815478141,0.0799627631508));
			contour_.push_back(Point2f(0.197184521859,0.0799627631508));
			contour_.push_back(Point2f(0.210550246872,0.0796648960881));
			contour_.push_back(Point2f(0.22390601742,0.0790693098467));
			contour_.push_back(Point2f(0.237245202685,0.0781763001206));
			contour_.push_back(Point2f(0.250561180083,0.0769863102677));
			contour_.push_back(Point2f(0.263847338551,0.0754999310894));
			contour_.push_back(Point2f(0.277097081833,0.073717900537));
			contour_.push_back(Point2f(0.290303831748,0.0716411033459));
			contour_.push_back(Point2f(0.303461031465,0.0692705705958));
			contour_.push_back(Point2f(0.31656214875,0.0666074791992));
			contour_.push_back(Point2f(0.329600679215,0.0636531513167));
			contour_.push_back(Point2f(0.342570149542,0.0604090537011));
			contour_.push_back(Point2f(0.355464120703,0.0568767969687));
			contour_.push_back(Point2f(0.36827619115,0.0530581347998));
			contour_.push_back(Point2f(0.381,0.0489549630683));	
			contour_.push_back(Point2f(0.381, 0));
			contour_.push_back(Point2f(0.381,0.0122875187586));
			contour_.push_back(Point2f(0.368470376004,0.0172542619015));
			contour_.push_back(Point2f(0.355811431603,0.0218814793796));
			contour_.push_back(Point2f(0.343032365293,0.0261658088713));
			contour_.push_back(Point2f(0.330142462857,0.0301041372113));
			contour_.push_back(Point2f(0.317151090612,0.0336936026522));
			contour_.push_back(Point2f(0.304067688612,0.0369315969449));
			contour_.push_back(Point2f(0.29090176378,0.0398157672328));
			contour_.push_back(Point2f(0.277662883004,0.0423440177624));
			contour_.push_back(Point2f(0.264360666186,0.0445145114055));
			contour_.push_back(Point2f(0.251004779249,0.0463256709944));
			contour_.push_back(Point2f(0.237604927115,0.0477761804682));
			contour_.push_back(Point2f(0.224170846654,0.0488649858284));
			contour_.push_back(Point2f(0.210712299607,0.0495912959055));
			contour_.push_back(Point2f(0.197239065494,0.0499545829336));
			contour_.push_back(Point2f(0.183760934506,0.0499545829336));
			contour_.push_back(Point2f(0.170287700393,0.0495912959055));
			contour_.push_back(Point2f(0.156829153346,0.0488649858284));
			contour_.push_back(Point2f(0.143395072885,0.0477761804682));
			contour_.push_back(Point2f(0.129995220751,0.0463256709944));
			contour_.push_back(Point2f(0.116639333814,0.0445145114055));
			contour_.push_back(Point2f(0.103337116996,0.0423440177624));
			contour_.push_back(Point2f(0.09009823622,0.0398157672328));
			contour_.push_back(Point2f(0.0769323113878,0.0369315969449));
			contour_.push_back(Point2f(0.0638489093876,0.0336936026522));
			contour_.push_back(Point2f(0.0508575371435,0.0301041372113));
			contour_.push_back(Point2f(0.0379676347069,0.0261658088713));
			contour_.push_back(Point2f(0.0251885683974,0.0218814793796));
			contour_.push_back(Point2f(0.0125296239964,0.0172542619015));
			contour_.push_back(Point2f(0,0.0122875187586));
			name_ = "bottom_boiler_tape";
			break;
		case 6: //target on the switch fence (2018)
			depth_ = 0;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.41));
			contour_.push_back(Point2f(0.0508,0.41));
			contour_.push_back(Point2f(0.0508,0));
			name_ = "plate_location_tape";
			break;
		case 7: //Cube (2018)
			depth_ = 0.3048;
			contour_.push_back(Point2f(0,0));
			contour_.push_back(Point2f(0,0.2794));
			contour_.push_back(Point2f(0.3302,0.2794));
			contour_.push_back(Point2f(0.2794,0));
			name_ = "cube";
			break;
		default:
			cerr << "error initializing object!" << endl;
	}

	computeProperties();

}

ObjectType::ObjectType(const vector< Point2f > &contour_in, const string &name_in, const float &depth_in) :
	contour_(contour_in),
	depth_(depth_in),
	name_(name_in)	
{
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point2f");
	computeProperties();
}

ObjectType::ObjectType(const vector< Point > &contour_in, const string &name_in, const float &depth_in):
	depth_(depth_in),
	name_(name_in)
{	
	if(contour_in.size() == 0 || name_in.length() == 0 || depth_in < 0)
		throw std::invalid_argument("bad argument to ObjectType Point");
	for (auto it = contour_in.cbegin(); it != contour_in.cend(); ++it)
		contour_.push_back(Point2f(it->x, it->y));
	computeProperties();
}

void ObjectType::computeProperties()
{
	float min_x = numeric_limits<float>::max();
	float min_y = numeric_limits<float>::max();
	float max_x = numeric_limits<float>::min();
	float max_y = numeric_limits<float>::min();
	for (auto it = contour_.cbegin(); it != contour_.cend(); ++it)
	{
		min_x = min(min_x, it->x);
		min_y = min(min_y, it->y);
		max_x = max(max_x, it->x);
		max_y = max(max_y, it->y);
	}
	width_ = max_x - min_x;
	height_ = max_y - min_y;
	area_ = contourArea(contour_);

	//compute moments and use them to find center of mass
	Moments mu = moments(contour_, false);
	com_ = Point2f(mu.m10 / mu.m00, mu.m01 / mu.m00);
}

Point3f ObjectType::screenToWorldCoords(const Rect &screen_position, double avg_depth, const Point2f &fov_size, const Size &frame_size, float cameraElevation) const
{
	/*
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
	*/

	// TODO : see about using camera params cx and cy here
	// Those will be the actual optical center of the frame
	Point2f rect_center(
			screen_position.x + (screen_position.width  / 2.0),
			screen_position.y + (screen_position.height / 2.0));
	Point2f dist_to_center(
			rect_center.x - (frame_size.width / 2.0),
			-rect_center.y + (frame_size.height / 2.0));

	// This uses formula from http://www.chiefdelphi.com/forums/showpost.php?p=1571187&postcount=4
	float azimuth = atanf(dist_to_center.x / (.5 * frame_size.width / tanf(fov_size.x / 2)));
	float inclination = atanf(dist_to_center.y / (.5 * frame_size.height / tanf(fov_size.y / 2))) - cameraElevation;

	// avg_depth is to front of object.  Add in half the
	// object's depth to move to the center of it
	avg_depth += depth_ / 2.;
	Point3f retPt(
			avg_depth * cosf(inclination) * sinf(azimuth),
			avg_depth * cosf(inclination) * cosf(azimuth),
			avg_depth * sinf(inclination));

#if 0
	cout << "screen pos: " << screen_position << endl;
	cout << "Rect center: " << rect_center << endl;
	cout << "Distance to center: " << dist_to_center << endl;
	cout << "Actual Azimuth: " << azimuth << endl;
	cout << "Actual Inclination: " << inclination << endl;
	cout << "Actual location: " << retPt << endl;
#endif
	return retPt;
}

Rect ObjectType::worldToScreenCoords(const Point3f &position, const Point2f &fov_size, const Size &frame_size, float cameraElevation) const
{
	// Object distance
	float r = sqrtf(position.x * position.x + position.y * position.y + position.z * position.z) - depth_ / 2.;

	// Add depth_/2 back in to r since position.z isn't adjusted for object depth
	// This will lead to a slightly inaccurate inclination
	// since we're dividing z from the center of the object
	// with r from the front of it
	float inclination = asinf(position.z / (r+depth_/2.)) + cameraElevation;

	float azimuth = asinf(position.x / sqrtf(position.x * position.x + position.y * position.y));

	//inverse of formula in screenToWorldCoords()
	Point2f dist_to_center(
			tanf(azimuth) * (0.5 * frame_size.width / tanf(fov_size.x / 2)),
			tanf(inclination) * (0.5 * frame_size.height / tanf(fov_size.y / 2)));
	Point2f rect_center(
			dist_to_center.x + (frame_size.width / 2.0),
			-dist_to_center.y + (frame_size.height / 2.0));

	Point2f angular_size(2.0 * atan2f(width_, 2.0*r), 2.0 * atan2f(height_, 2.0*r));
	Point2f screen_size(
			angular_size.x * (frame_size.width / fov_size.x),
			angular_size.y * (frame_size.height / fov_size.y));

	Point topLeft(
			cvRound(rect_center.x - (screen_size.x / 2.0)),
			cvRound(rect_center.y - (screen_size.y / 2.0)));

#if 0
	cout << "r = " << r << endl;
	cout << "azimuth = " << azimuth << endl;
	cout << "inclination = " << inclination << endl;

	cout << "Distance to center: " << dist_to_center << endl;
	cout << "rect_center " << rect_center << endl;
	cout << "angular_size " << angular_size << endl;
	cout << "screen_size " << screen_size << endl;
	cout << "worldToScreenCoords " << Rect(topLeft.x, topLeft.y, cvRound(screen_size.x), cvRound(screen_size.y)) << endl;
#endif
	return Rect(topLeft.x, topLeft.y, cvRound(screen_size.x), cvRound(screen_size.y));
}

float ObjectType::expectedDepth(const Rect &screen_position, const Size &frame_size, const float hfov) const
{
	// TODO : use larger of width, height for slightly better resolution
	float percent_image = (float)screen_position.width / frame_size.width;
	float size_fov      = percent_image * hfov;
	return width_ / (2.0 * tanf(size_fov / 2.0)) - depth_ / 2.;
}

bool ObjectType::operator== (const ObjectType &t1) const 
{
	return this->shape() == t1.shape();
}

