#pragma once

#define PRINT_AND_THROW(s) do {\
  std::cerr << "\033[1;31mERROR " << s << "\033[0m\n";\
  std::cerr << "at " << __FILE__ << ":" << __LINE__ << std::endl;\
  std::stringstream ss;\
  ss << s;\
  throw std::runtime_error(ss.str());\
} while (0)
#define FAIL_IF_FALSE(expr) if (!(expr)) {\
    PRINT_AND_THROW( "expected true: " #expr);\
  }

#ifdef __CDT_PARSER__
#define BOOST_FOREACH(a,b) for(a;;)
#endif

#define ALWAYS_ASSERT(exp) if (!(exp)) {printf("%s failed in file %s at line %i\n", #exp, __FILE__, __LINE__ ); abort();}
