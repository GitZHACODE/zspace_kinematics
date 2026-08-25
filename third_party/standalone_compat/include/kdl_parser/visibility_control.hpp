#pragma once

// The vendored parser is linked statically, so Windows DLL import/export
// annotations from the ament packaging are intentionally not applicable.
#define KDL_PARSER_PUBLIC
#define KDL_PARSER_PUBLIC_TYPE
#define KDL_PARSER_LOCAL
