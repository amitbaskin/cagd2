#include <cagd.h>
#include <stdio.h>
#include "resource.h"
#include "menus.h"
#include <mmsystem.h>

#pragma comment(lib, "winmm.lib")

#if defined(_WIN32)
    #if _MSC_VER >= 1900
  #pragma comment(lib, "legacy_stdio_definitions.lib")
    #endif
#endif

enum {
  MY_CLICK = CAGD_USER,
  MY_POLY,
  MY_ANIM,
  MY_DRAG,
  MY_ADD,
  MY_COLOR,
  MY_REMOVE,
};

char *animText[] = {
  "Animation Demo",
  "During the animation you can freely\n"
  "rotate, translate and scale the scene."
};


HMENU myPopup;
UINT myText;
char myBuffer[BUFSIZ];

void myMessage(PSTR title, PSTR message, UINT type)
{
  MessageBox(cagdGetWindow(), message, title, MB_OK | MB_APPLMODAL | type);
}

void myTimer(int x, int y, PVOID userData)
{
  cagdRotate(2, 0, 1, 0);
  cagdRedraw();
}

int main(int argc, char *argv[])
{
  cagdBegin( "CAGD", 800, 800 );
  init_menus();

  PlaySound( TEXT( "see-you-later-203103.wav" ), NULL, SND_FILENAME | SND_ASYNC );

  cagdShowHelp();
  cagdMainLoop();

  return 0;
}
