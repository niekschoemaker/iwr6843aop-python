import curses
import time

def report_progress(filename, progress):
    """progress: 0-10"""
    stdscr.addstr(0, 0, "Moving file: {0}".format(filename))
    stdscr.addstr(1, 0, "Total progress: [{1:10}] {0}%".format(progress * 10, "#" * progress))
    stdscr.refresh()

if __name__ == "__main__":
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()

    try:
        for i in range(10):
            report_progress("file_{0}.txt".format(i), i+1)
            time.sleep(0.5)
    finally:
        curses.echo()
        curses.nocbreak()
        curses.endwin()