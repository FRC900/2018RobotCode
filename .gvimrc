" Vim
" An example for a gvimrc file.
" The commands in this are executed when the GUI is started.
"
" To use it, copy it to
"     for Unix and OS/2:  ~/.gvimrc
"             for Amiga:  s:.gvimrc
"  for MS-DOS and Win32:  $VIM\_gvimrc

" Make external commands work through a pipe instead of a pseudo-tty
"set noguipty

" set the X11 font to use
set guifont=Courier\ 10\ Pitch\ 10

" Make command line two lines high
set ch=2

" Only do this for Vim version 5.0 and later.
if version >= 500

  " I like highlighting strings inside C comments
  "unlet c_comment_strings

  " Switch on syntax highlighting.
  " Switch on syntax highlighting if it wasn't on yet.
  if !exists("syntax_on")
    syntax on
  endif

  " Switch on search pattern highlighting.
  set hlsearch

  " Hide the mouse pointer while typing
  set mousehide
  
  set sm
  set guioptions-=m

  set vb t_vb=

  " Set nice colors
  colorscheme kjaget

endif
