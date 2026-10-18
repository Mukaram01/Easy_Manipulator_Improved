set pagination off
set confirm off
set breakpoint pending on
handle SIGPIPE nostop noprint
handle SIG32 nostop noprint
handle SIG33 nostop noprint
set $armed = 0
break QWebEngineView::~QWebEngineView
commands 1
  silent
  if $armed == 0
    set $armed = 1
    set $view = (void *)$rdi
    set $d = *(void **)((char *)$view + 0x30)
    printf "[watch] ~QWebEngineView this=%p d_ptr=%p d->page=%p\n", $view, $d, *(void **)((char *)$d + 8)
    watch -l *(long *)((char *)$d + 8)
    commands
      silent
      printf "\n[watch] write to freed d->page field, new value:\n"
      p/x *(long *)((char *)$d + 8)
      bt 16
      continue
    end
  end
  continue
end
run
quit
