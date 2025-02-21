# OpenSTM

## .nvim.lua

```
local lsp = require("lspconfig")
lsp.clangd.setup({
	cmd = { "clangd", "--query-driver=/opt/homebrew/bin/arm-none-eabi-g++" },
})
```
