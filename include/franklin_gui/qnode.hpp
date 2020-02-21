/**
 * @file /include/franklin_gui/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef franklin_gui_QNODE_HPP_
#define franklin_gui_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

// To workaround boost/qt4 problems that won't be bugfixed. Refer to
//    https://bugreports.qt.io/browse/QTBUG-22829
#ifndef Q_MOC_RUN
#include <ros/ros.h>
#endif
#include <string>
#include <QThread>
#include <QStringListModel>
#include <std_msgs/Float32.h>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace franklin_gui {

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
	bool init();
	bool init(const std::string &master_url, const std::string &host_url);
	void run();
  void sendTargetPos(double pX, double pY, double pT);
  void sendStop(bool b);
  void info_dest_Callback(const std_msgs::Float32 msg);
	int progressData;

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
	void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
	void loggingUpdated();
  void rosShutdown();
  void progressDataS();

private:
	int init_argc;
	char** init_argv;
	ros::Publisher chatter_publisher;
  QStringListModel logging_model;

  ros::Publisher pub_dest;
  ros::Publisher pub_stop;
  ros::Subscriber sub_info_dest;

};

}  // namespace franklin_gui

#endif /* franklin_gui_QNODE_HPP_ */
