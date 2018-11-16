#include "CocoaBridge.h"

#import <Cocoa/Cocoa.h>

void CocoaBridge::setAllowsAutomaticWindowTabbing( const bool allow )
{
  [NSWindow setAllowsAutomaticWindowTabbing: allow];
}
