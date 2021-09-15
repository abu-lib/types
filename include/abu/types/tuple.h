// Copyright 2021 Francois Chabot

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ABU_TYPES_TUPLE
#define ABU_TYPES_TUPLE

#include <tuple>
#include <type_traits>

namespace abu {

// abu::tuple
// It's exactly like std::tuple, but can be used as a structural type

namespace details_ {
template <class T>
struct unwrap_refwrapper {
  using type = T;
};

template <class T>
struct unwrap_refwrapper<std::reference_wrapper<T>> {
  using type = T&;
};

template <class T>
using unwrap_decay_t =
    typename unwrap_refwrapper<typename std::decay<T>::type>::type;

template <std::size_t I, typename T>
struct tuple_leaf {
  template <typename U>
  constexpr tuple_leaf& operator=(const U& rhs) {
    data_ = rhs.data_;
    return *this;
  }

  template <typename U>
  constexpr tuple_leaf& operator=(U&& rhs) {
    data_ = std::move(rhs.data_);
    return *this;
  }

  [[no_unique_address]] T data_;
};

template <typename Seq, typename...>
struct tuple_;

template <std::size_t I, typename T>
std::type_identity<T> tuple_element_lookup(const tuple_leaf<I, T>&);

template <std::size_t I, typename Tuple>
using tuple_element = decltype(tuple_element_lookup<I>(std::declval<Tuple>()));

template <std::size_t I, typename Tuple>
using tuple_element_t = typename tuple_element<I, Tuple>::type;

template <std::size_t I, typename T, typename Enable = void>
struct maybe_tuple_element : tuple_element<I, T> {};

template <std::size_t I, typename Seq, typename... Ts>
struct maybe_tuple_element<I,
                           tuple_<Seq, Ts...>,
                           std::enable_if_t<(I >= sizeof...(Ts))>>
    : std::type_identity<void> {};

template <std::size_t I, typename Tuple>
using maybe_tuple_element_t = typename maybe_tuple_element<I, Tuple>::type;

template <typename T, std::size_t I>
constexpr T& get(tuple_leaf<I, T>& t) noexcept {
  return t.data_;
}

template <typename T, std::size_t I>
constexpr const T& get(const tuple_leaf<I, T>& t) noexcept {
  return t.data_;
}

template <typename T, std::size_t I>
constexpr T&& get(tuple_leaf<I, T>&& t) noexcept {
  return std::move(t.data_);
}

template <typename T, std::size_t I>
constexpr const T&& get(const tuple_leaf<I, T>&& t) noexcept {
  return std::move(t.data_);
}

template <std::size_t I, typename T>
constexpr T& get(tuple_leaf<I, T>& t) noexcept {
  return t.data_;
}

template <std::size_t I, typename T>
constexpr const T& get(const tuple_leaf<I, T>& t) noexcept {
  return t.data_;
}

template <std::size_t I, typename T>
constexpr T&& get(tuple_leaf<I, T>&& t) noexcept {
  return std::move(t.data_);
}

template <std::size_t I, typename T>
constexpr const T&& get(const tuple_leaf<I, T>&& t) noexcept {
  return std::move(t.data_);
}

template <std::size_t... Is, typename... Ts>
struct tuple_<std::index_sequence<Is...>, Ts...> : tuple_leaf<Is, Ts>... {
  static_assert(sizeof...(Is) == sizeof...(Ts));

  constexpr tuple_() : tuple_leaf<Is, Ts>{Ts{}}... {}

  constexpr tuple_(const Ts&... args) requires(
      sizeof...(Ts) >= 1 && (std::is_copy_constructible_v<Ts> && ...))
      : tuple_leaf<Is, Ts>{args}... {}

  template <typename... Us>
  constexpr tuple_(Us&&... args) requires(sizeof...(Ts) >= 1 &&
                                          sizeof...(Us) == sizeof...(Ts) &&
                                          (std::is_constructible_v<Ts, Us&&> &&
                                           ...))
      : tuple_leaf<Is, Ts>{Ts(std::forward<Us>(args))}... {}

  template <typename... Us>
  constexpr tuple_(
      const tuple_<std::index_sequence<Is...>, Us...>&
          other) requires(sizeof...(Ts) >= 1 &&
                          sizeof...(Us) == sizeof...(Ts) &&
                          (std::is_constructible_v<Ts, const Us&> && ...) &&
                          (sizeof...(Ts) != 1 ||
                           (!(std::is_convertible_v<const tuple_<Us>&, Ts> &&
                              ...) &&
                            !(std::is_constructible_v<Ts, const tuple_<Us>&> &&
                              ...) &&
                            !(std::is_same_v<Ts, Us> && ...))))
      : tuple_leaf<Is, Ts>{Ts(get<Is>(other))}... {}

  template <typename... Us>
  constexpr tuple_(
      const tuple_<std::index_sequence<Is...>, Us...>&&
          other) requires(sizeof...(Ts) >= 1 &&
                          sizeof...(Us) == sizeof...(Ts) &&
                          (std::is_constructible_v<Ts, Us&&> && ...) &&
                          (sizeof...(Ts) != 1 ||
                           (!(std::is_convertible_v<tuple_<Us>, Ts> && ...) &&
                            !(std::is_constructible_v<Ts, tuple_<Us>> && ...) &&
                            !(std::is_same_v<Ts, Us> && ...))))
      : tuple_leaf<Is, Ts>{std::move(get<Is>(other))}... {}

  constexpr tuple_(const tuple_& other) = default;
  constexpr tuple_(tuple_&& other) = default;

  tuple_& operator=(const tuple_& rhs) = default;
  tuple_& operator=(tuple_&& rhs) noexcept(
      (std::is_nothrow_move_assignable_v<Ts> && ...)) = default;

  template <class... Us>
  constexpr tuple_& operator=(const tuple_<Us...>& other) {
    ((get<Is>(*this) = get<Is>(other)), ...);
    return *this;
  }

  template <class... Us>
  constexpr tuple_& operator=(tuple_<Us...>&& other) {
    ((get<Is>(*this) = std::move(get<Is>(other))), ...);
    return *this;
  }

  void swap(tuple_& other) {
    (swap(get<Is>(*this), get<Is>(other)), ...);
  }
};

template <std::size_t... Is, typename... Ts, typename... Us>
constexpr bool operator==(
    const tuple_<std::index_sequence<Is...>, Ts...>& lhs,
    const tuple_<std::index_sequence<Is...>, Us...>& rhs) {
  return ((get<Is>(lhs) == get<Is>(rhs)) && ...);
}

template <std::size_t I,
          std::size_t N,
          typename Res,
          typename LhsT,
          typename RhsT>
constexpr Res perform_tuple_compare(const LhsT& lhs, const RhsT& rhs) {
  auto tmp = get<I>(lhs) <=> get<I>(rhs);
  if constexpr (I == N - 1) {
    return tmp;
  } else {
    if (tmp != 0) {
      return tmp;
    }

    return perform_tuple_compare<I + 1, N, Res>(lhs, rhs);
  }
}

template <std::size_t... Is, typename... Ts, typename... Us>
constexpr auto operator<=>(
    const tuple_<std::index_sequence<Is...>, Ts...>& lhs,
    const tuple_<std::index_sequence<Is...>, Us...>& rhs) {
  using result_type =
      std::common_comparison_category_t<decltype(get<Is>(lhs) <=>
                                                 get<Is>(rhs))...>;
  return perform_tuple_compare<0, sizeof...(Ts), result_type>(lhs, rhs);
}

template<typename T>
concept DefaultListConstructible = requires { 
  [](T) {}({});
};
}  // namespace details_
template <typename... Ts>
struct tuple
    : details_::tuple_<std::make_index_sequence<sizeof...(Ts)>, Ts...> {
 private:
  using impl_type =
      details_::tuple_<std::make_index_sequence<sizeof...(Ts)>, Ts...>;


  static constexpr bool expl_1 = !(details_::DefaultListConstructible<Ts> && ...);
  static constexpr bool expl_2 = !(std::is_convertible_v<const Ts&, Ts> && ...);

  template <typename... Us>
  static constexpr bool expl_3 = !(std::is_convertible_v<Us&&, Ts> && ...);

  template <typename... Us>
  static constexpr bool expl_4 = !(std::is_convertible_v<const Us&, Ts> && ...);

  template <typename... Us>
  static constexpr bool expl_5 = !(std::is_convertible_v<Us, Ts> && ...);

  template <typename U1, typename U2>
  static constexpr bool expl_6 =
      !std::is_convertible_v<const U1&,
                             details_::maybe_tuple_element_t<0, impl_type>> ||
      !std::is_convertible_v<const U2&,
                             details_::maybe_tuple_element_t<1, impl_type>>;

  template <typename U1, typename U2>
  static constexpr bool expl_7 =
      !std::is_convertible_v<U1&&,
                             details_::maybe_tuple_element_t<0, impl_type>> ||
      !std::is_convertible_v<U2&&,
                             details_::maybe_tuple_element_t<1, impl_type>>;

 public:
  // constructor (1)
  constexpr explicit(expl_1)
      tuple() requires(std::is_default_constructible_v<Ts>&&...)
      : impl_type{} {}

  // constructor (2)
  constexpr explicit(expl_2) tuple(const Ts&... args) requires(
      sizeof...(Ts) >= 1 && (std::is_copy_constructible_v<Ts> && ...))
      : impl_type{args...} {}

  // constructor (3)
  template <typename... Us>
  constexpr explicit(expl_3<Us...>)
      tuple(Us&&... args) requires(sizeof...(Ts) >= 1 &&
                                   sizeof...(Us) == sizeof...(Ts) &&
                                   (std::is_constructible_v<Ts, Us&&> && ...))
      : impl_type{std::forward<Us>(args)...} {}

  // constructor (4)
  template <typename... Us>
  constexpr explicit(expl_4<Us...>) tuple(const tuple<Us...>& other) requires(
      sizeof...(Ts) >= 1 && sizeof...(Us) == sizeof...(Ts) &&
      (std::is_constructible_v<Ts, const Us&> && ...) &&
      (sizeof...(Ts) != 1 ||
       (!(std::is_convertible_v<const tuple<Us>&, Ts> && ...) &&
        !(std::is_constructible_v<Ts, const tuple<Us>&> && ...) &&
        !(std::is_same_v<Ts, Us> && ...))))
      : impl_type{other} {}

  // constructor (5)
  template <typename... Us>
  constexpr explicit(expl_5<Us...>) tuple(const tuple<Us...>&& other) requires(
      sizeof...(Ts) >= 1 && sizeof...(Us) == sizeof...(Ts) &&
      (std::is_constructible_v<Ts, Us&&> && ...) &&
      (sizeof...(Ts) != 1 ||
       (!(std::is_convertible_v<tuple<Us>, Ts> && ...) &&
        !(std::is_constructible_v<Ts, tuple<Us>> && ...) &&
        !(std::is_same_v<Ts, Us> && ...))))
      : impl_type{std::move(other)} {}

  // constructor (6)
  template <class U1, class U2>
  constexpr explicit(expl_6<U1, U2>) tuple(const std::pair<U1, U2>& p) requires(
      sizeof...(Ts) == 2 &&
      std::is_constructible_v<details_::tuple_element_t<0, impl_type>,
                              const U1&> &&
      std::is_constructible_v<details_::tuple_element_t<1, impl_type>,
                              const U2&>)
      : impl_type{p.first, p.second} {}

  // constructor (7)
  template <class U1, class U2>
  constexpr explicit(expl_7<U1, U2>) tuple(std::pair<U1, U2>&& p) requires(
      sizeof...(Ts) == 2 &&
      std::is_constructible_v<details_::tuple_element_t<0, impl_type>, U1&&> &&
      std::is_constructible_v<details_::tuple_element_t<1, impl_type>, U2&&>)
      : impl_type{std::forward<U1>(p.first), std::forward<U2>(p.second)} {}

  // constructor (8)
  constexpr tuple(const tuple& other) = default;
  // constructor (9)
  constexpr tuple(tuple&& other) = default;

  // operator=() (1)
  constexpr tuple& operator=(const tuple& rhs) requires(
      !(std::is_copy_assignable_v<Ts> && ...)) = delete;
  constexpr tuple& operator=(const tuple& rhs) requires(
      (std::is_copy_assignable_v<Ts> && ...)) = default;

  // operator=() (2)
  constexpr tuple& operator=(tuple&& rhs) noexcept(
      (std::is_nothrow_move_assignable_v<Ts> &&
       ...)) requires((std::is_move_assignable_v<Ts> && ...)) = default;

  // operator=() (3)
  template <class... Us>
  constexpr tuple& operator=(const tuple<Us...>& other) requires(
      sizeof...(Ts) == sizeof...(Us) &&
      (std::is_assignable_v<Ts&, const Us&> && ...)) {
    static_cast<impl_type&>(*this) = static_cast<const impl_type&>(other);
    return *this;
  }

  // operator=() (4)
  template <class... Us>
  constexpr tuple& operator=(tuple<Us...>&& other) requires(
      sizeof...(Ts) == sizeof...(Us) &&
      (std::is_assignable_v<Ts&, Us> && ...)) {
    static_cast<impl_type&>(*this) = static_cast<impl_type&&>(other);
    return *this;
  }

  // operator=() (5)
  template <typename U1, typename U2>
  constexpr tuple& operator=(const std::pair<U1, U2>& p) requires(
      sizeof...(Ts) == 2 &&
      std::is_assignable_v<details_::tuple_element_t<0, impl_type>&,
                           const U1&> &&
      std::is_assignable_v<details_::tuple_element_t<1, impl_type>&,
                           const U2&>) {
    get<0>(*this) = p.first;
    get<1>(*this) = p.second;
    return *this;
  }

  // operator=() (6)
  template <typename U1, typename U2>
  constexpr tuple& operator=(std::pair<U1, U2>&& p) requires(
      sizeof...(Ts) == 2 &&
      std::is_assignable_v<details_::tuple_element_t<0, impl_type>&, U1> &&
      std::is_assignable_v<details_::tuple_element_t<1, impl_type>&, U2>) {
    get<0>(*this) = std::forward<U1>(p.first);
    get<1>(*this) = std::forward<U2>(p.second);

    return *this;
  }

  // swap
  constexpr void swap(tuple& other) noexcept(
      (noexcept(swap(std::declval<Ts&>(), std::declval<Ts&>())) && ...)) {
    impl_type::swap(other);
  }
};

// make_tuple
template <typename... Ts>
constexpr tuple<details_::unwrap_decay_t<Ts>...> make_tuple(Ts&&... args) {
  return tuple<details_::unwrap_decay_t<Ts>...>{std::forward<Ts>(args)...};
}

// tie
template <typename... Ts>
constexpr tuple<Ts&...> tie(Ts&... args) noexcept {
  return {args...};
}

// forward_as_tuple
template <typename... Ts>
constexpr tuple<Ts&&...> forward_as_tuple(Ts&&... args) noexcept {
  return tuple<Ts&&...>(std::forward<Ts>(args)...);
}

// tuple_cat

namespace details_ {

template <typename Lhs, typename Rhs>
struct tuple_cat_merger;

template <typename... LhsTs, typename... RhsTs>
struct tuple_cat_merger<tuple<LhsTs...>, tuple<RhsTs...>> {
  using value_type = tuple<LhsTs..., RhsTs...>;

  template <std::size_t... Is, std::size_t... Js>
  static constexpr value_type merge(
      const details_::tuple_<std::index_sequence<Is...>, LhsTs...>& lhs,
      const details_::tuple_<std::index_sequence<Js...>, RhsTs...>& rhs) {
    return value_type{get<Is>(lhs)..., get<Js>(rhs)...};
  }

  template <std::size_t... Is, std::size_t... Js>
  static constexpr value_type merge(
      details_::tuple_<std::index_sequence<Is...>, LhsTs...>&& lhs,
      const details_::tuple_<std::index_sequence<Js...>, RhsTs...>& rhs) {
    return value_type{get<Is>(lhs)..., get<Js>(rhs)...};
  }

  template <std::size_t... Is, std::size_t... Js>
  static constexpr value_type merge(
      const details_::tuple_<std::index_sequence<Is...>, LhsTs...>& lhs,
      details_::tuple_<std::index_sequence<Js...>, RhsTs...>&& rhs) {
    return value_type{get<Is>(lhs)..., get<Js>(rhs)...};
  }

  template <std::size_t... Is, std::size_t... Js>
  static constexpr value_type merge(
      details_::tuple_<std::index_sequence<Is...>, LhsTs...>&& lhs,
      details_::tuple_<std::index_sequence<Js...>, RhsTs...>&& rhs) {
    return value_type{get<Is>(lhs)..., get<Js>(rhs)...};
  }
};

}  // namespace details_

template <typename... Tups>
constexpr tuple<> tuple_cat(Tups&&... args) {
  return {};
}

template <typename First, typename... Tups>
constexpr First tuple_cat(First&& first, Tups&&... rest) {
  return std::forward<First>(first);
}

template <typename First, typename Second, typename... Tups>
constexpr auto tuple_cat(First&& first, Second&& second, Tups&&... rest) {
  using merger_type =
      details_::tuple_cat_merger<std::decay_t<First>, std::decay_t<Second>>;

  auto tmp = merger_type::merge(std::forward<First>(first),
                                std::forward<Second>(second));

  return tuple_cat(std::move(tmp), std::forward<Tups...>(rest)...);
}

// get
template <std::size_t I, typename... Ts>
constexpr auto& get(tuple<Ts...>& t) noexcept {
  return details_::get<I>(t);
}

template <std::size_t I, typename... Ts>
constexpr const auto& get(const tuple<Ts...>& t) noexcept {
  return details_::get<I>(t);
}

template <std::size_t I, typename... Ts>
constexpr auto&& get(tuple<Ts...>&& t) noexcept {
  return details_::get<I>(std::move(t));
}

template <std::size_t I, typename... Ts>
constexpr const auto& get(const tuple<Ts...>&& t) noexcept {
  return details_::get<I>(std::move(t));
}

template <typename T, typename... Ts>
constexpr auto& get(tuple<Ts...>& t) noexcept {
  return details_::get<T>(t);
}

template <typename T, typename... Ts>
constexpr const auto& get(const tuple<Ts...>& t) noexcept {
  return details_::get<T>(t);
}

template <typename T, typename... Ts>
constexpr auto&& get(tuple<Ts...>&& t) noexcept {
  return details_::get<T>(std::move(t));
}

template <typename T, typename... Ts>
constexpr const auto& get(const tuple<Ts...>&& t) noexcept {
  return details_::get<T>(std::move(t));
}

template <typename... Ts, typename... Us>
constexpr bool operator==(const tuple<Ts...>& lhs, const tuple<Us...>& rhs) {
  return static_cast<
             const details_::tuple_<std::make_index_sequence<sizeof...(Ts)>,
                                    Ts...>&>(lhs) ==
         static_cast<
             const details_::tuple_<std::make_index_sequence<sizeof...(Us)>,
                                    Ts...>&>(rhs);
}

template <typename... Ts, typename... Us>
constexpr auto operator<=>(const tuple<Ts...>& lhs, const tuple<Us...>& rhs) {
  return static_cast<
             const details_::tuple_<std::make_index_sequence<sizeof...(Ts)>,
                                    Ts...>&>(lhs) <=>
         static_cast<
             const details_::tuple_<std::make_index_sequence<sizeof...(Us)>,
                                    Us...>&>(rhs);
}

template <typename T>
struct tuple_size;

template <typename... Ts>
struct tuple_size<tuple<Ts...>>
    : std::integral_constant<std::size_t, sizeof...(Ts)> {};

template <std::size_t I, typename T>
struct tuple_element;

template <std::size_t I, class... Ts>
struct tuple_element<I, tuple<Ts...>>
    : details_::tuple_element<
          I,
          details_::tuple_<std::make_index_sequence<sizeof...(Ts)>, Ts...>> {};

}  // namespace abu


// In order to allow structured bindings
namespace std {
template <typename... Ts>
struct tuple_size<::abu::tuple<Ts...>> : 
  ::abu::tuple_size<::abu::tuple<Ts...>> {};

template <std::size_t I, typename... Ts>
struct tuple_element<I, ::abu::tuple<Ts...>>
    : abu::tuple_element<I, ::abu::tuple<Ts...>> {};

}
#endif