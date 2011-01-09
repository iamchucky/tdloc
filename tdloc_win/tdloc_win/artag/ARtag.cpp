#include "ARtag.h"


ARtag::ARtag()  : id_(-1), pose_age_(0)
{
	pose_ = cvCreateMat( 4, 4, CV_32F);
}

ARtag::~ARtag()
{
}

void ARtag::setId(unsigned int id)
{
	id_ = id;
}

unsigned int ARtag::getId() const
{
	return id_;
}

void ARtag::setPose(CvMat * pose)
{
	for (int i = 0; i < pose->rows; ++i)
		for (int j = 0; j < pose->cols; ++j)
			CV_MAT_ELEM(*pose_ , float, i, j) = CV_MAT_ELEM(*pose , float, i, j);
}

const CvMat * ARtag::getPose() const
{
	return pose_;
}

void ARtag::setPoseAge(unsigned int age)
{
	pose_age_ = age;
}

unsigned int ARtag::getPoseAge() const
{
	return pose_age_;
}