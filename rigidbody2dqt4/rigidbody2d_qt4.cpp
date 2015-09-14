#include <QApplication>
#include <QDesktopWidget>

#include "Window.h"

#include <iostream>

static void centerWindow( Window& window )
{
  const QDesktopWidget* const desktop{ QApplication::desktop() };

  const int screen_width{ desktop->screenGeometry().width() };
  const int screen_height{ desktop->screenGeometry().height() };

  const QSize window_size{ window.size() };
  const int width{ window_size.width() };
  const int height{ window_size.height() };

  const int x{ ( screen_width - width ) / 2 };
  const int y{ ( screen_height - height ) / 2 };

  window.move( x, y );
}

int main( int argc, char** argv )
{
  QApplication app{ argc, argv };
  const QStringList arguments{ app.arguments() };
  if( arguments.count() > 2 )
  {
    std::cerr << "Error, must provide a valid configuration file name or no argument. Exiting." << std::endl;
    return EXIT_FAILURE;
  }
  Window window{ arguments.count() == 2 ? arguments[1] : "" };
  window.resize( window.sizeHint() );
  window.setWindowTitle( "2D Rigid Body Simulation" );
  centerWindow( window );
  window.show();
  window.raise();
  return app.exec();
}
