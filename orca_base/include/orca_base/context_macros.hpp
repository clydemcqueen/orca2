#ifndef ORCA_BASE_CONTEXT_MACROS_HPP
#define ORCA_BASE_CONTEXT_MACROS_HPP

#define CXT_PARAM_FIELD_DEF(n, d, t) t n##_ = d;

#define CXT_PARAM_LOAD_PARAM(n, d, t) node.get_parameter<t>(#n, n##_);

// TODO catch type exceptions
#define CXT_PARAM_CHANGE_PARAM(n, d, t) \
if (parameter.get_name() == #n) {\
  n##_ = parameter.get_value<t>(); \
  std::cout << "change value of " << #n << " to " << n##_ << std::endl; \
}

#endif // ORCA_BASE_CONTEXT_MACROS_HPP
