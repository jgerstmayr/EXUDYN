(sec-usersettings)=
# User settings that persist between runs

A setting you change on every start — the output directory, the multi-sampling of the renderer, the
size of the basis vectors — can be stored once, in `~/.exudyn/config.json`, and taken by every run
afterwards. `import exudyn` reads that file once into `exudyn.special.overrideSettings`; nothing
writes it by itself.

It belongs to the Exudyn module, so it is documented with the module:
**[](#sec-overridesettings)** — the file and its sections, what may be stored, how a dialog
remembers its size, and **[](#sec-environmentvariables)**, the environment variables that change
what Exudyn does before a script says anything.

The functions that read and write the file are in `exudyn.misc.overrideSettings`.
