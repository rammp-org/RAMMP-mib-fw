#pragma once
// Declarations only: the planner's fmt::formatter specializations compile, but
// nothing formats because the stub logger never calls them.
namespace fmt {
template <typename T, typename Char = char, typename Enable = void> struct formatter;
template <typename OutputIt, typename... Args>
OutputIt format_to(OutputIt out, const char *, Args &&...);
} // namespace fmt
