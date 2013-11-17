#include "testApp.h"

using namespace ofxCv;
using namespace cv;


const float dyingTime = 5;

JointTracker::JointTracker()
{
	colors[0] = ofColor(255,0,0);
	colors[1] = ofColor(0,255,0);
	colors[2] = ofColor(0,0,255);
}

bool JointTracker::addPoint(int _label)
{
	if(jointLabels.size() < 3)
	{
		jointLabels.push_back(_label);
		return true;
	}
	return false;
}

void JointTracker::reset()
{
	jointLabels.clear();
}

void JointTracker::draw(ofxCv::RectTrackerFollower <Fixture> * _trackerReference)
{
	vector<Fixture>& followers = _trackerReference->getFollowers();
	for(int i = 0; i < jointLabels.size(); i++)
	{
		ofVec2f point;
		if(_trackerReference->existsCurrent(jointLabels[i]))
		{
			point = toOf(_trackerReference->getCurrent(jointLabels[i])).getCenter();
			previous[i] = point;
		}
		else
		{
			point = previous[i];
		}
		//const vector <unsigned int> labels = _trackerReference->getCurrentLabels();
		//ofVec2f point = ofVec2f(0,0);
		ofSetColor(colors[i]); 
		ofCircle(point, 10);
	}
}

float JointTracker::getAngle()
{
	if(jointLabels.size() < 3) return -1;
	ofVec2f vecA, vecB;
	//2-1, 0-1
	vecA = previous[0]-previous[1];
	vecA.normalize();
	vecB = previous[2]-previous[1];
	vecB.normalize();
	cout << ofRadToDeg(acos(vecA.dot(vecB))) << endl;
	return vecA.dot(vecB);
}

void Fixture::setup(const cv::Rect& track) {
	color.setHsb(ofRandom(0, 255), 255, 255);
	cur = toOf(track).getCenter();
	smooth = cur;
}

void Fixture::update(const cv::Rect& track) {
	cur = toOf(track).getCenter();
	smooth.interpolate(cur, .5);
	all.addVertex(smooth);
}

void Fixture::kill() {
	float curTime = ofGetElapsedTimef();
	if(startedDying == 0) {
		startedDying = curTime;
	} else if(curTime - startedDying > dyingTime) {
		dead = true;
	}
}

void Fixture::draw() {
	ofPushStyle();
	float size = 16;
	ofSetColor(0,0,200);
	if(startedDying) {
		ofSetColor(ofColor::red);
		size = ofMap(ofGetElapsedTimef() - startedDying, 0, dyingTime, size, 0, true);
	}
	ofNoFill();
	ofCircle(cur, size);
	ofSetColor(color);
	all.draw();
	ofSetColor(255);
	ofDrawBitmapString(ofToString(label), cur);
	ofPopStyle();
}
//--------------------------------------------------------------
void testApp::setup(){
	
	ofDisableAlphaBlending(); //Kinect alpha channel is default 0;
	ofSetFrameRate(30);
	ofSetBackgroundAuto(false);

	kinect.initSensor();
	kinect.initColorStream(640, 480, true);
	kinect.start();

	setGUI1(); 

	ofSetBackgroundAuto(false);
	trackingColorMode = TRACK_COLOR_RGB;
	targetColor = kinect.getColorPixelsRef().getColor(0, 0);
	targetColor.r = 254;
	targetColor.g = 254;
	targetColor.b = 254;
	targetColor.a = 0;
	threshold = 4.4;
	contourFinder.setMinAreaRadius(2);
	contourFinder.setMaxAreaRadius(20);
	contourFinder.setThreshold(threshold);
	contourFinder.setTargetColor(targetColor, trackingColorMode);
	contourFinder.findContours(toCv(kinect.getColorPixelsRef()));

	tracker.setPersistence(60);
	tracker.setMaximumDistance(100);
	trackerRef = &tracker;

	picked = NULL;
	trackedLimb = new JointTracker();
	doDebugDraw = true;
}

//--------------------------------------------------------------
void testApp::update(){
	ofBackground(205);
	kinect.update();

	contourFinder.setThreshold(threshold);
	//contourFinder.setTargetColor(targetColor, trackingColorMode);
	contourFinder.findContours(toCv(kinect.getColorPixelsRef()));
	tracker.track(contourFinder.getBoundingRects());
}

//--------------------------------------------------------------
void testApp::draw()
{
	ofSetColor(255);
	kinect.draw(0,0);
	trackedLimb->draw(trackerRef);

	if(doDebugDraw)
	{
		for(int i =0; i < contourFinder.size();i++)
		{
			ofSetColor(magentaPrint);
			ofPolyline minAreaRect = toOf(contourFinder.getMinAreaRect(i));
			minAreaRect.draw();
		}
	}

	ofSetColor(0);
	//cout << targetColor << endl;
	ofSetColor(255,0,0);

	if(picked != NULL)
	{
		ofCircle(picked->getCur(), 5);
	}

	if(!(ofGetFrameNum() % 20))	trackedLimb->getAngle();
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){

	bool bAltPressed = (bool) ((GetKeyState(VK_MENU) & 0x80) > 0);
    bool bShiftPressed = (bool) ((GetKeyState(VK_SHIFT) & 0x80) > 0);
    bool bControlPressed = (bool) ((GetKeyState(VK_CONTROL) & 0x80) > 0);

	if(bControlPressed && key == 'a')
	{
		targetColor = kinect.getColorPixelsRef().getColor(mouseX, mouseY);
		contourFinder.setTargetColor(targetColor, trackingColorMode);
	}

	switch (key)
	{
		case 'h':
			gui1->toggleVisible();
			break;
		case 'r':
			trackedLimb->reset();
			break;
		case 'd':
			doDebugDraw = ! doDebugDraw;
		default:
			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

	if(mouseX < 640 && mouseY < 480)
	{
		vector<Fixture>& followers = tracker.getFollowers();
		if(followers.size())
		{
			float minDist=0xffffffff;
			ofVec2f closest;
			int index  = 0;
			for(int i =0; i < followers.size(); i++)
			{
				ofVec2f centroid = followers[i].getCur();
				float currentDist = ofDist(x, y, centroid.x, centroid.y);
				if(currentDist < minDist)
				{
					minDist = currentDist;
					picked = & followers[i];
				}
			}
		}
		if(trackedLimb != NULL)
		{
			int newLabel = picked->getLabel();
			trackedLimb->addPoint(newLabel);
		}
	}
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}

void testApp::guiEvent(ofxUIEventArgs &e)
{
	string name = e.widget->getName(); 
	int kind = e.widget->getKind(); 
	//cout << "got event from: " << name << endl; 	
	
	if(name == "THRESHOLD")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;  
		threshold = slider->getScaledValue();
		return;
	}
	if(name == "MIN_SIZE")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;  
		contourFinder.setMinAreaRadius(slider->getScaledValue()); 
		return;
	}
	if(name == "MAX_SIZE")
	{
		ofxUISlider *slider = (ofxUISlider *) e.widget;  
		contourFinder.setMaxAreaRadius(slider->getScaledValue());
		return;
	}
}

void testApp::setGUI1()
{	
	float dim = 16; 
	float xInit = OFX_UI_GLOBAL_WIDGET_SPACING; 
    float length = 255-xInit; 
	
    vector<string> names; 
	names.push_back("RAD1");
	names.push_back("RAD2");
	names.push_back("RAD3");	
	
	gui1 = new ofxUICanvas(640, 0, 384, ofGetHeight()); 
	gui1->addWidgetDown(new ofxUILabel("iMitor Mobility Monitor", OFX_UI_FONT_LARGE)); 
    gui1->addWidgetDown(new ofxUILabel("Press 'h' to hide", OFX_UI_FONT_LARGE)); 

    gui1->addSpacer(length-xInit, 2);
	gui1->addWidgetDown(new ofxUILabel("H SLIDERS", OFX_UI_FONT_MEDIUM)); 
	gui1->addSlider("THRESHOLD", 0.0, 255.0, threshold, length-xInit, dim);
	gui1->addSlider("MIN_SIZE", 0.0, 20.0, 2, length-xInit,dim);
	gui1->addSlider("MAX_SIZE", 0.0, 100.0, 20, length-xInit,dim);

    gui1->addSpacer(length-xInit, 2); 
    gui1->addWidgetDown(new ofxUILabel("NOT_ALLOCATED", OFX_UI_FONT_MEDIUM)); 
	gui1->addSlider("0", 0.0, 255.0, 150, dim, 160);
	gui1->setWidgetPosition(OFX_UI_WIDGET_POSITION_RIGHT);
	gui1->addSlider("1", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("2", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("3", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("4", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("5", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("6", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("7", 0.0, 255.0, 150, dim, 160);
	gui1->addSlider("8", 0.0, 255.0, 150, dim, 160);
	gui1->setWidgetPosition(OFX_UI_WIDGET_POSITION_DOWN);
    
    gui1->addSpacer(length-xInit, 2);
	gui1->addRadio("RADIO HORIZONTAL", names, OFX_UI_ORIENTATION_HORIZONTAL, dim, dim); 
	gui1->addRadio("RADIO VERTICAL", names, OFX_UI_ORIENTATION_VERTICAL, dim, dim); 

    gui1->addSpacer(length-xInit, 2);
	gui1->addWidgetDown(new ofxUILabel("BUTTONS", OFX_UI_FONT_MEDIUM)); 
	gui1->addButton("DRAW GRID", false, dim, dim);
	gui1->addWidgetDown(new ofxUILabel("TOGGLES", OFX_UI_FONT_MEDIUM)); 
	gui1->addToggle( "D_GRID", false, dim, dim);
    
    gui1->addSpacer(length-xInit, 2);
    gui1->addWidgetDown(new ofxUILabel("RANGE SLIDER", OFX_UI_FONT_MEDIUM)); 
	gui1->addRangeSlider("RSLIDER", 0.0, 255.0, 50.0, 100.0, length-xInit,dim);

	ofAddListener(gui1->newGUIEvent,this,&testApp::guiEvent);
}