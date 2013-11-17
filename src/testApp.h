#pragma once

#include "ofMain.h"
#include "ofxKinectCommonBridge.h"
#include "ofxCv.h"
#include "ofxUI.h"


class Fixture : public ofxCv::RectFollower {
private:
	ofColor color;
	ofVec2f cur, smooth;
	float startedDying;
	ofPolyline all;
public:
	Fixture()
		:startedDying(0) {
	}
	void setup(const cv::Rect& track);
	void update(const cv::Rect& track);
	void kill();
	void draw();
	ofVec2f getCur(){return cur;}
};

class JointTracker
{
public:
	JointTracker();
	bool addPoint(int);
	void draw(ofxCv::RectTrackerFollower <Fixture> * _trackerRef);
	void reset();
	float getAngle();
private:
	vector < int > jointLabels;
	ofColor colors [3];
	ofVec2f previous[3];
};

class testApp : public ofBaseApp{
	public:
		void setup();
		void update();
		void draw();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
		void guiEvent(ofxUIEventArgs &e);

		ofxKinectCommonBridge kinect;

		ofxUICanvas *gui1;
		void setGUI1();

		ofxCv::ContourFinder contourFinder;
		float threshold;
		ofxCv::TrackingColorMode trackingColorMode;
		ofColor targetColor;
		ofxCv::RectTrackerFollower <Fixture> tracker;
		ofxCv::RectTrackerFollower <Fixture> * trackerRef;

		ofPoint picker;
		Fixture * picked;

		JointTracker * trackedLimb;

		bool doDebugDraw;
};
