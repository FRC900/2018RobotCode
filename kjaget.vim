" local syntax file - set colors on a per-machine basis:
" vim: tw=0 ts=4 sw=4
" Vim color file

hi clear
set background=dark
if exists("syntax_on")
  syntax reset
endif
let g:colors_name = "kjaget"
  highlight Comment gui=NONE guifg=yellow guibg=black
  highlight Normal guibg=black guifg=white
  highlight Constant guifg=cyan guibg=black
  highlight Special gui=NONE guifg=cyan guibg=black
  highlight PreProc guibg=black guifg=red
  highlight Statement guifg=white
  highlight MoreMsg guifg=green
  highlight Identifier guifg=green
  highlight Type guifg=white

hi Scrollbar	  guifg=darkcyan guibg=cyan
hi Menu			  guifg=black guibg=cyan
hi SpecialKey	  term=bold  cterm=bold  ctermfg=darkred  guifg=Blue
hi NonText		  term=bold  cterm=bold  ctermfg=darkred  gui=bold	guifg=Blue
hi Directory	  term=bold  cterm=bold  ctermfg=brown	guifg=Blue
hi ErrorMsg		  term=standout  cterm=bold  ctermfg=grey  ctermbg=blue  guifg=White  guibg=Red
"hi Search		  term=reverse	ctermfg=white  ctermbg=red	guifg=white  guibg=Red
"hi ModeMsg		  term=bold  cterm=bold  gui=bold  guifg=White	guibg=Blue
hi LineNr		  term=underline  cterm=bold  ctermfg=darkcyan	guifg=Yellow
hi Question		  term=standout  cterm=bold  ctermfg=darkgreen	gui=bold  guifg=Green
hi StatusLine	  term=bold,reverse  cterm=bold ctermfg=lightblue ctermbg=white gui=bold guifg=blue guibg=white
hi StatusLineNC   term=reverse	ctermfg=white ctermbg=lightblue guifg=white guibg=blue
hi Title		  term=bold  cterm=bold  ctermfg=darkmagenta  gui=bold	guifg=Magenta
hi Visual		  term=reverse	cterm=reverse  gui=reverse
hi WarningMsg	  term=standout  cterm=bold  ctermfg=darkblue  guifg=Red
"hi Cursor		  guifg=bg	guibg=Green
hi Error		  term=reverse	ctermfg=darkcyan  ctermbg=black  guifg=Red	guibg=Black
hi Todo			  term=standout  ctermfg=black	ctermbg=darkcyan  guifg=Blue  guibg=Yellow
hi link IncSearch		Visual
hi link String			Constant
hi link Character		Constant
hi link Number			Constant
hi link Boolean			Constant
hi link Float			Number
hi link Function		Identifier
hi link Conditional		Statement
hi link Repeat			Statement
hi link Label			Statement
hi link Operator		Statement
hi link Keyword			Statement
hi link Exception		Statement
hi link Include			PreProc
hi link Define			PreProc
hi link Macro			PreProc
hi link PreCondit		PreProc
hi link StorageClass	Type
hi link Structure		Type
hi link Typedef			Type
hi link Tag				Special
hi link SpecialChar		Special
hi link Delimiter		Special
hi link SpecialComment	Special
hi link Debug			Special
