void __libc_init_array(void) { }

int _close(int fd)              { (void)fd; return -1; }
int _lseek(int fd, int off, int w) { (void)fd; (void)off; (void)w; return -1; }
int _read(int fd, char *b, int c)  { (void)fd; (void)b; (void)c; return -1; }
int _write(int fd, const char *b, int c) { (void)fd; (void)b; return c; }
int _fstat(int fd, void *st)        { (void)fd; (void)st; return -1; }
int _isatty(int fd)             { (void)fd; return 1; }
