// Comment {{{

/* This program  reads the angles from the accelerometer and gyroscope
	on a BerryIMU connected to a Raspberry Pi.
	http://ozzmaker.com/


	Copyright (C) 2014  Mark Williams

	This library is free software; you can redistribute it and/or
	modify it under the terms of the GNU Library General Public
	License as published by the Free Software Foundation; either
	version 2 of the License, or (at your option) any later version.
	This library is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
	Library General Public License for more details.
	You should have received a copy of the GNU Library General Public
	License along with this library; if not, write to the Free
	Software Foundation, Inc., 59 Temple Place - Suite 330, Boston,
	MA 02111-1308, USA
*/

// }}}
// Includes {{{

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Vector3.h"
#include "tf2_msgs/TFMessage.h"
#include "tf/tfMessage.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <unistd.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "quaternion.c"

#include "genmarks/glove.h"

// }}}
// Main Code Definitions {{{

#define DT 0.02		 // [s/loop] loop period.

#define RAD_TO_DEG 57.29577951308232
#define DEG_TO_RAD 0.017453292519943295

//#define M_PI 3.14159265358989323846

// }}}
// Conversion stuff {{{

float deg2rad(float deg)
{
	return deg / 180.0 * M_PI;
}

geometry_msgs::PoseStamped quat2posest(quaternion_t q)
{
	geometry_msgs::Quaternion geoqtmsg;
	geoqtmsg.x = q.x;
	geoqtmsg.y = q.y;
	geoqtmsg.z = q.z;
	geoqtmsg.w = q.w;

	geometry_msgs::Pose posemsg;
	posemsg.orientation = geoqtmsg;
	geometry_msgs::PoseStamped posestmsg;
	posestmsg.pose = posemsg;
	//posestmsg.header.frame_id = "/base_link";
	posestmsg.header.frame_id = "/world";

	return posestmsg;
}

quaternion_t pose2quat (geometry_msgs::Pose p) {
	quaternion_t q;

	q.w = p.orientation.w;
	q.x = p.orientation.x;
	q.y = p.orientation.y;
	q.z = p.orientation.z;

	return q;
}

// typedef struct {
// 	double x;
// 	double y;
// 	double z;
// } xyz_t;

const float CHROMA_MAX = 1.5f;

int marker_ids = 0;
visualization_msgs::Marker genmarker(xyz_t posn, xyz_t scale, quaternion_t q, float chroma) {
	visualization_msgs::Marker marker;
	//marker.header.frame_id = "/base_link";
	marker.header.frame_id = "/world";
	marker.header.stamp = ros::Time::now();
	marker.ns = "genmarks_finger";
	//marker.id = marker_ids++;
	marker.type = visualization_msgs::Marker::CYLINDER;
	//marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation = quat2posest(q).pose.orientation;
	marker.pose.position.x = posn.x;
	marker.pose.position.y = posn.y;
	marker.pose.position.z = posn.z;
	marker.scale.x = scale.x;
	marker.scale.y = scale.y;
	marker.scale.z = scale.z;
	if (chroma > CHROMA_MAX) {
		chroma = CHROMA_MAX;
	}
	if (chroma < 0.0f) {
		chroma = 0.0f;
	}
	marker.color.r = chroma / CHROMA_MAX;
	marker.color.g = 1.0f - (chroma / CHROMA_MAX);
	marker.color.b = 0.0f;
	marker.color.a = 1.0f;
	//marker.lifetime = ros::Duration();

	return marker;
}

// }}}
// Math fns {{{

void prepBus() {
	usleep(300000);
}

xyz_t compute_endpt(xyz_t startpt, xyz_t base_vector, quaternion_t rotation) {
	return xyz_add(startpt, quaternion_rotate(rotation, base_vector));
}

xyz_t compute_midpt(xyz_t startpt, xyz_t base_vector, quaternion_t rotation) {
	xyz_t half_base = xyz_scale(base_vector, 0.5);
	return xyz_add(startpt, quaternion_rotate(rotation, half_base));
}

ros::Subscriber ros_sub_forcedata;
ros::Subscriber ros_sub_genmarks_raw;
ros::Publisher ros_pub_genmarks;
ros::Publisher ros_pub_pose_0;
ros::Publisher ros_pub_pose_2;
ros::Publisher ros_pub_pose_8;

// }}}
// Timing stuff {{{

void INThandler(int sig)
{
		signal(sig, SIG_IGN);
		exit(0);
}

int mymillis()
{
	struct timeval tv;
	gettimeofday(&tv, NULL);
	return (tv.tv_sec) * 1000 + (tv.tv_usec)/1000;
}

int timeval_subtract(struct timeval *result, struct timeval *t2, struct timeval *t1)
{
	long int diff = (t2->tv_usec + 1000000 * t2->tv_sec) - (t1->tv_usec + 1000000 * t1->tv_sec);
	result->tv_sec = diff / 1000000;
	result->tv_usec = diff % 1000000;
	return (diff<0);
}

// }}}
// Origin defs {{{

quaternion_t origin0;
quaternion_t origin1;
quaternion_t origin2;
quaternion_t origin3;
quaternion_t origin4;
quaternion_t origin5;
quaternion_t origin6;
quaternion_t origin7;
quaternion_t origin8;
quaternion_t origin9;
quaternion_t originA;
quaternion_t originB;
quaternion_t originC;
quaternion_t originD;
quaternion_t originE;
quaternion_t originF;

quaternion_t cache0;
quaternion_t cache1;
quaternion_t cache2;
quaternion_t cache3;
quaternion_t cache4;
quaternion_t cache5;
quaternion_t cache6;
quaternion_t cache7;
quaternion_t cache8;
quaternion_t cache9;
quaternion_t cacheA;
quaternion_t cacheB;
quaternion_t cacheC;
quaternion_t cacheD;
quaternion_t cacheE;
quaternion_t cacheF;


// }}}
// ROS CB {{{

// Force stuff {{{

void ros_cb_imuctrl (const std_msgs::String::ConstPtr& msgs) {
	printf("Got it.\n");

	origin0 = cache0;
	origin1 = cache1;
	origin2 = cache2;
	origin3 = cache3;
	origin4 = cache4;
	origin5 = cache5;
	origin6 = cache6;
	origin7 = cache7;
	origin8 = cache8;
	origin9 = cache9;
	originA = cacheA;
	originB = cacheB;
	originC = cacheC;
	originD = cacheD;
	originE = cacheE;
	originF = cacheF;
}

float forces[8];

void ros_cb_forcedata (const genmarks::glove msg) {
	forces[0] = msg.segments[0].force[0];
	forces[1] = msg.segments[0].force[1];
	forces[2] = msg.segments[1].force[1];
	forces[3] = msg.segments[2].force[0];
	forces[4] = msg.segments[2].force[1];
	forces[5] = msg.segments[3].force[0];
	forces[6] = msg.segments[3].force[1];
}

// }}}

quaternion_t relative0;
quaternion_t relative1;
quaternion_t relative2;
quaternion_t relative3;
quaternion_t relative4;
quaternion_t relative5;
quaternion_t relative6;
quaternion_t relative7;
quaternion_t relative8;
quaternion_t relative9;
quaternion_t relativeA;
quaternion_t relativeB;
quaternion_t relativeC;
quaternion_t relativeD;
quaternion_t relativeE;


void ros_cb_raw (const geometry_msgs::PoseArray::ConstPtr& recposearray) {
	relative0 = pose2quat(recposearray->poses[0x0]);
	relative1 = pose2quat(recposearray->poses[0x1]);
	relative2 = pose2quat(recposearray->poses[0x2]);
	relative3 = pose2quat(recposearray->poses[0x3]);
	relative4 = pose2quat(recposearray->poses[0x4]);
	relative5 = pose2quat(recposearray->poses[0x5]);
	relative6 = pose2quat(recposearray->poses[0x6]);
	relative7 = pose2quat(recposearray->poses[0x7]);
	relative8 = pose2quat(recposearray->poses[0x8]);
	relative9 = pose2quat(recposearray->poses[0x9]);
	relativeA = pose2quat(recposearray->poses[0xA]);
	relativeB = pose2quat(recposearray->poses[0xB]);
	relativeC = pose2quat(recposearray->poses[0xC]);
	relativeD = pose2quat(recposearray->poses[0xD]);
	relativeE = pose2quat(recposearray->poses[0xE]);
	//quaternion_t relativeF recaternion_mul(sampleF, quaternion_inv(originF));

	// Do whatever with the samples {{{

	// Dims and constants {{{

	xyz_t ORIG_PALM = {0.0, 0.0, 0.0};
	xyz_t DIMS_PALM = {2.0, 2.0, 0.2};

	xyz_t BASE_PROXPHALANGE = {0, 0, 1};
	xyz_t DIMS_PROXPHALANGE = {0.35, 0.35, 1.0};

	xyz_t BASE_MEDPHALANGE = {0, 0, 0.8};
	xyz_t DIMS_MEDPHALANGE = {0.35, 0.35, 0.8};

	xyz_t BASE_DISTALPHALANGE = {0, 0, 0.4};
	xyz_t DIMS_DISTALPHALANGE = {0.35, 0.35, 0.4};

	quaternion_t FORWARD = quaternion_from_axis_angle(AXIS_Y, M_PI / 2);

	// }}}

	// Palm {{{

	visualization_msgs::Marker palm = genmarker(ORIG_PALM, DIMS_PALM, relative0, forces[5]);

	// }}}

	// Thumb base {{{

	xyz_t phi_s_thumbbase = quaternion_rotate(
		quaternion_from_axis_angle(AXIS_Z, 0.40),
		AXIS_Y
	);
	quaternion_t phi_q_thumbbase = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, 0.40),
		quaternion_from_axis_angle(AXIS_Y, M_PI / 2 + 0.35)
	);

	quaternion_t psi_q_thumbbase = quaternion_mul(relative1, phi_q_thumbbase);
	xyz_t psi_s_thumbbase = quaternion_rotate(relative0, phi_s_thumbbase);

	//xyz_t tmpbase = {-0.38941834230865047, 0.9210609940028852, 0.0};
	//quaternion_t tmpquat = {0.6930117232058354, -0.14048043101898117, 0.6930117232058353, 0.1404804310189812};
	visualization_msgs::Marker thumbbase = genmarker(
		compute_midpt(psi_s_thumbbase, BASE_PROXPHALANGE, psi_q_thumbbase),
		//tmpbase,
		DIMS_PROXPHALANGE,
		psi_q_thumbbase,
		forces[6]
	);

	// }}}
	// Thumb tip {{{

	quaternion_t phi_q_thumbtip = quaternion_mul(quaternion_from_axis_angle(AXIS_Y, 0.35), FORWARD);
	xyz_t psi_s_thumbtip = compute_endpt(psi_s_thumbbase, BASE_PROXPHALANGE, psi_q_thumbbase);
	quaternion_t psi_q_thumbtip = quaternion_mul(relative2, phi_q_thumbtip);

	visualization_msgs::Marker thumbtip = genmarker(
		compute_midpt(psi_s_thumbtip, BASE_MEDPHALANGE, psi_q_thumbtip),
		DIMS_MEDPHALANGE,
		psi_q_thumbtip,
		forces[0]
	);

	// }}}

	// Index base {{{

	quaternion_t phi_q_indexbase = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, 0.15),
		FORWARD
	);
	xyz_t phi_s_indexbase = quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, 0.57), AXIS_X);

	quaternion_t psi_q_indexbase = quaternion_mul(relative3, phi_q_indexbase);
	xyz_t psi_s_indexbase = quaternion_rotate(relative0, phi_s_indexbase);

	visualization_msgs::Marker indexbase = genmarker(
		compute_midpt(psi_s_indexbase, BASE_PROXPHALANGE, psi_q_indexbase),
		DIMS_PROXPHALANGE,
		psi_q_indexbase,
		0.0f
	);

	// }}}
	// Index med {{{

	quaternion_t phi_q_indexmed = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, 0.15),
		FORWARD
	);

	xyz_t psi_s_indexmed = compute_endpt(psi_s_indexbase, BASE_PROXPHALANGE, psi_q_indexbase);
	quaternion_t psi_q_indexmed = quaternion_mul(relative4, phi_q_indexmed);

	visualization_msgs::Marker indexmed = genmarker(
		compute_midpt(psi_s_indexmed, BASE_MEDPHALANGE, psi_q_indexmed),
		DIMS_MEDPHALANGE,
		psi_q_indexmed,
		forces[4]
	);

	// }}}
	// Index tip {{{

	quaternion_t phi_q_indextip = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, 0.15),
		FORWARD
	);

	xyz_t psi_s_indextip = compute_endpt(psi_s_indexmed, BASE_MEDPHALANGE, psi_q_indexmed);
	quaternion_t psi_q_indextip = quaternion_mul(relative5, phi_q_indextip);

	visualization_msgs::Marker indextip = genmarker(
		compute_midpt(psi_s_indextip, BASE_DISTALPHALANGE, psi_q_indextip),
		DIMS_DISTALPHALANGE,
		psi_q_indextip,
		forces[1]
	);

	// }}}

	// Middle base {{{

	quaternion_t phi_q_middlebase = FORWARD;
	xyz_t phi_s_middlebase = quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, 0.00), AXIS_X);

	quaternion_t psi_q_middlebase = quaternion_mul(relative6, phi_q_middlebase);
	xyz_t psi_s_middlebase = quaternion_rotate(relative0, phi_s_middlebase);

	visualization_msgs::Marker middlebase = genmarker(
		compute_midpt(psi_s_middlebase, BASE_PROXPHALANGE, psi_q_middlebase),
		DIMS_PROXPHALANGE,
		psi_q_middlebase,
		0.0f
	);

	// }}}
	// Middle med {{{

	quaternion_t phi_q_middlemed = FORWARD;
	xyz_t psi_s_middlemed = compute_endpt(psi_s_middlebase, BASE_PROXPHALANGE, psi_q_middlebase);
	quaternion_t psi_q_middlemed = quaternion_mul(relative7, phi_q_middlemed);

	visualization_msgs::Marker middlemed = genmarker(
		compute_midpt(psi_s_middlemed, BASE_MEDPHALANGE, psi_q_middlemed),
		DIMS_MEDPHALANGE,
		psi_q_middlemed,
		forces[3]
	);

	// }}}
	// Middle tip {{{

	quaternion_t phi_q_middletip = FORWARD;
	xyz_t psi_s_middletip = compute_endpt(psi_s_middlemed, BASE_MEDPHALANGE, psi_q_middlemed);
	quaternion_t psi_q_middletip = quaternion_mul(relative8, phi_q_middletip);

	visualization_msgs::Marker middletip = genmarker(
		compute_midpt(psi_s_middletip, BASE_DISTALPHALANGE, psi_q_middletip),
		DIMS_DISTALPHALANGE,
		psi_q_middletip,
		forces[2]
	);

	// }}}

	// Ring base {{{

	quaternion_t phi_q_ringbase = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.15),
		FORWARD
	);
	xyz_t phi_s_ringbase = quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, -0.35), AXIS_X);

	quaternion_t psi_q_ringbase = quaternion_mul(relative9, phi_q_ringbase);
	xyz_t psi_s_ringbase = quaternion_rotate(relative0, phi_s_ringbase);

	visualization_msgs::Marker ringbase = genmarker(
		compute_midpt(psi_s_ringbase, BASE_PROXPHALANGE, psi_q_ringbase),
		DIMS_PROXPHALANGE,
		psi_q_ringbase,
		0.0f
	);

	// }}}
	// Ring med {{{

	quaternion_t phi_q_ringmed = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.15),
		FORWARD
	);

	xyz_t psi_s_ringmed = compute_endpt(psi_s_ringbase, BASE_PROXPHALANGE, psi_q_ringbase);
	quaternion_t psi_q_ringmed = quaternion_mul(relativeA, phi_q_ringmed);

	visualization_msgs::Marker ringmed = genmarker(
		compute_midpt(psi_s_ringmed, BASE_MEDPHALANGE, psi_q_ringmed),
		DIMS_MEDPHALANGE,
		psi_q_ringmed,
		0.0f
	);

	// }}}
	// Ring tip {{{

	quaternion_t phi_q_ringtip = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.15),
		FORWARD
	);

	xyz_t psi_s_ringtip = compute_endpt(psi_s_ringmed, BASE_MEDPHALANGE, psi_q_ringmed);
	quaternion_t psi_q_ringtip = quaternion_mul(relativeB, phi_q_ringtip);

	visualization_msgs::Marker ringtip = genmarker(
		compute_midpt(psi_s_ringtip, BASE_DISTALPHALANGE, psi_q_ringtip),
		DIMS_DISTALPHALANGE,
		psi_q_ringtip,
		0.0f
	);

	// }}}

	// Pinkie base {{{

	quaternion_t phi_q_pinkiebase = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.335),
		FORWARD
	);
	xyz_t phi_s_pinkiebase = quaternion_rotate(quaternion_from_axis_angle(AXIS_Z, -0.75), AXIS_X);

	quaternion_t psi_q_pinkiebase = quaternion_mul(relativeC, phi_q_pinkiebase);
	xyz_t psi_s_pinkiebase = quaternion_rotate(relative0, phi_s_pinkiebase);

	visualization_msgs::Marker pinkiebase = genmarker(
		compute_midpt(psi_s_pinkiebase, BASE_MEDPHALANGE, psi_q_pinkiebase),
		DIMS_MEDPHALANGE,
		psi_q_pinkiebase,
		0.0f
	);

	// }}}
	// Pinkie med {{{

	quaternion_t phi_q_pinkiemed = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.25),
		FORWARD
	);

	xyz_t psi_s_pinkiemed = compute_endpt(psi_s_pinkiebase, BASE_MEDPHALANGE, psi_q_pinkiebase);
	quaternion_t psi_q_pinkiemed = quaternion_mul(relativeD, phi_q_pinkiemed);

	visualization_msgs::Marker pinkiemed = genmarker(
		compute_midpt(psi_s_pinkiemed, BASE_MEDPHALANGE, psi_q_pinkiemed),
		DIMS_MEDPHALANGE,
		psi_q_pinkiemed,
		0.0f
	);

	// }}}
	// Pinkie tip {{{

	quaternion_t phi_q_pinkietip = quaternion_mul(
		quaternion_from_axis_angle(AXIS_Z, -0.25),
		FORWARD
	);

	xyz_t psi_s_pinkietip = compute_endpt(psi_s_pinkiemed, BASE_MEDPHALANGE, psi_q_pinkiemed);
	quaternion_t psi_q_pinkietip = quaternion_mul(relativeE, phi_q_pinkietip);

	visualization_msgs::Marker pinkietip = genmarker(
		compute_midpt(psi_s_pinkietip, BASE_DISTALPHALANGE, psi_q_pinkietip),
		DIMS_DISTALPHALANGE,
		psi_q_pinkietip,
		0.0f
	);

	// }}}

	// Set ids {{{

	palm.id = 0x0;
	thumbbase.id = 0x1;
	thumbtip.id = 0x2;
	indexbase.id = 0x3;
	indexmed.id = 0x4;
	indextip.id = 0x5;
	middlebase.id = 0x6;
	middlemed.id = 0x7;
	middletip.id = 0x8;
	ringbase.id = 0x9;
	ringmed.id = 0xA;
	ringtip.id = 0xB;
	pinkiebase.id = 0xC;
	pinkiemed.id = 0xD;
	pinkietip.id = 0xE;

	// }}}

	// Publish {{{

	visualization_msgs::MarkerArray markarray;
	markarray.markers.resize(15);
	markarray.markers[0x0] = palm;
	markarray.markers[0x1] = thumbbase;
	markarray.markers[0x2] = thumbtip;
	markarray.markers[0x3] = indexbase;
	markarray.markers[0x4] = indexmed;
	markarray.markers[0x5] = indextip;
	markarray.markers[0x6] = middlebase;
	markarray.markers[0x7] = middlemed;
	markarray.markers[0x8] = middletip;
	markarray.markers[0x9] = ringbase;
	markarray.markers[0xA] = ringmed;
	markarray.markers[0xB] = ringtip;
	markarray.markers[0xC] = pinkiebase;
	markarray.markers[0xD] = pinkiemed;
	markarray.markers[0xE] = pinkietip;
	ros_pub_genmarks.publish(markarray);

	geometry_msgs::PoseStamped glove_pose_0;
        glove_pose_0.header.frame_id = "/world";
        glove_pose_0.pose = recposearray->poses[0x0];
        ros_pub_pose_0.publish(glove_pose_0);

	geometry_msgs::PoseStamped glove_pose_2;
        glove_pose_2.header.frame_id = "/world";
        glove_pose_2.pose = recposearray->poses[0x2];
        ros_pub_pose_2.publish(glove_pose_2);

	geometry_msgs::PoseStamped glove_pose_8;
        glove_pose_8.header.frame_id = "/world";
        glove_pose_8.pose = recposearray->poses[0x8];
        ros_pub_pose_8.publish(glove_pose_8);
	// }}}

	// }}}
}

// }}}
// Main {{{

int main(int argc, char * argv[])
{
	// Set up ROS and timing stuff {{{

	ros::init(argc, argv, "genmarks");

	ros::NodeHandle ros_n;

	ros_sub_forcedata = ros_n.subscribe("/glove_sensors", 1000, ros_cb_forcedata);
	//ros_sub_genmarks_raw = ros_n.subscribe("/tac_glove_imutracker_raw", 1000, ros_cb_raw);
	ros_sub_genmarks_raw = ros_n.subscribe("/tac_glove_imutracker_rel", 1000, ros_cb_raw);
	ros_pub_genmarks = ros_n.advertise<visualization_msgs::MarkerArray>("/tac_glove_genmarks", 1000);
        ros_pub_pose_0 = ros_n.advertise<geometry_msgs::PoseStamped>("/tac_glove_pose_0", 1000);
        ros_pub_pose_2 = ros_n.advertise<geometry_msgs::PoseStamped>("/tac_glove_pose_2", 1000);
        ros_pub_pose_8 = ros_n.advertise<geometry_msgs::PoseStamped>("/tac_glove_pose_8", 1000);

	signal(SIGINT, INThandler);

	int startInt  = mymillis();
	struct  timeval tvBegin, tvEnd,tvDiff;


	// }}}


	while (ros::ok()) {

		//Each loop should be at least DT seconds
		while(mymillis() - startInt < (DT*1000))
		{
			usleep(100);
		}

		ros::spinOnce();
	}
}

// }}}
