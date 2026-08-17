// Sends a BACKSPACE key press to the MuJoCo simulate window, which triggers
// mj_resetData in unitree_mujoco (see simulate/src/main.cc user_key_cb).
// Used by the Gate 1 smoke test to put the robot back into its spawn pose
// (free fall) without restarting the simulator process.
//
// Build: cc -O2 tools/x11_sim_reset.c -o build/x11_sim_reset -lX11 -lXtst
// Exit 0 if the key was delivered, 1 otherwise.
#include <stdio.h>
#include <string.h>
#include <X11/Xlib.h>
#include <X11/keysym.h>
#include <X11/extensions/XTest.h>

static Window find_mujoco(Display* dpy, Window root) {
  Window parent, *children = NULL;
  unsigned int n = 0;
  if (!XQueryTree(dpy, root, &root, &parent, &children, &n)) return 0;
  Window found = 0;
  for (unsigned int i = 0; i < n && !found; ++i) {
    char* name = NULL;
    if (XFetchName(dpy, children[i], &name) && name) {
      if (strstr(name, "MuJoCo") != NULL) found = children[i];
      XFree(name);
    }
    if (!found) found = find_mujoco(dpy, children[i]);
  }
  if (children) XFree(children);
  return found;
}

int main(void) {
  Display* dpy = XOpenDisplay(NULL);
  if (!dpy) {
    fprintf(stderr, "x11_sim_reset: cannot open DISPLAY\n");
    return 1;
  }
  Window win = find_mujoco(dpy, DefaultRootWindow(dpy));
  if (!win) {
    fprintf(stderr, "x11_sim_reset: no MuJoCo window found\n");
    XCloseDisplay(dpy);
    return 1;
  }
  XSetInputFocus(dpy, win, RevertToParent, CurrentTime);
  KeyCode kc = XKeysymToKeycode(dpy, XK_BackSpace);
  XTestFakeKeyEvent(dpy, kc, True, CurrentTime);
  XFlush(dpy);
  XTestFakeKeyEvent(dpy, kc, False, CurrentTime);
  XFlush(dpy);
  XCloseDisplay(dpy);
  return 0;
}
