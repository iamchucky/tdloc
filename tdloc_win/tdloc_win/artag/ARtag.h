#ifndef ARTAG_H
#define ARTAG_H 
#include "opencv\cv.h"
#include "opencv\highgui.h"

class ARtag
{
	public:
		ARtag();
		virtual ~ARtag();

		void setId(unsigned int id);
		unsigned int getId() const;

		void setPose(CvMat * pose);
		const CvMat * getPose() const;

		void setPoseAge(unsigned int age);
		unsigned int getPoseAge() const;

	protected:
		unsigned int id_;
		CvMat * pose_;
		unsigned int pose_age_;
};

#endif