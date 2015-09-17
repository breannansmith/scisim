#ifndef WINDOW_H
#define WINDOW_H

#include <QMainWindow>

class ContentWidget;

class Window final : public QMainWindow
{

public:

  Window( const QString& scene_name = "", QWidget* parent = nullptr );

  void keyPressEvent( QKeyEvent* event );

protected:

  void closeEvent( QCloseEvent* event );

private:

  ContentWidget* m_content_widget;

};

#endif
