#ifndef ORCA_BASE_CONTEXT_MACROS_HPP
#define ORCA_BASE_CONTEXT_MACROS_HPP

#define CXT_PARAM_FIELD_DEF(n, d, t) t n##_ = d;

#define CXT_PARAM_LOAD_PARAM(n, d, t) node.get_parameter<t>(#n, n##_);

#endif // ORCA_BASE_CONTEXT_MACROS_HPP
