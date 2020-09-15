#ifndef SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT
#define SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT

#include <boost/scope_exit.hpp>
#include <scenario_runner/syntax/storyboard_element_state.hpp>

namespace scenario_runner { inline namespace syntax
{
  template <typename T>
  struct StoryboardElement
  {
    Object state { standby_state };

    const UnsignedInteger maximum_execution_count;

    UnsignedInteger execution_count { 0 };

    explicit constexpr StoryboardElement(UnsignedInteger maximum_execution_count = 1)
      : maximum_execution_count { maximum_execution_count }
    {}

    const auto& currentState() const
    {
      return state;
    }

    #define BOILERPLATE(NAME, STATE)                                           \
    constexpr auto NAME() const noexcept                                       \
    {                                                                          \
      return currentState().template as<StoryboardElementState>(__FILE__, __LINE__) == StoryboardElementState::STATE; \
    } static_assert(true, "")

    BOILERPLATE(standby, standbyState);
    BOILERPLATE(starting, startTransition);
    BOILERPLATE(running, runningState);
    BOILERPLATE(ending, endTransition);
    BOILERPLATE(complete, completeState);
    BOILERPLATE(stopping, stopTransition);
    BOILERPLATE(skipping, skipTransition);

    #undef BOILERPLATE

    static constexpr void start() noexcept
    {}

    Object override()
    {
      if (not complete() and not stopping())
      {
        return state = stop_transition;
      }
      else
      {
        return state;
      }
    }

  private:
    template <typename... Ts>
    constexpr decltype(auto) ready(Ts&&... xs) const
    {
      return static_cast<const T&>(*this).ready(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    constexpr decltype(auto) accomplished(Ts&&... xs) const
    {
      return static_cast<const T&>(*this).accomplished(std::forward<decltype(xs)>(xs)...);
    }

    template <typename... Ts>
    constexpr decltype(auto) stopTriggered(Ts&&... xs) const
    {
      return static_cast<const T&>(*this).stopTriggered(std::forward<decltype(xs)>(xs)...);
    }

  protected:
    template <typename U, typename Node, typename Scope>
    decltype(auto) makeStoryboardElement(const Node& node, Scope& inner_scope)
    {
      const auto name { readAttribute<String>(node, inner_scope, "name") };

      const auto result {
        inner_scope.storyboard_elements.emplace(
          name.empty() ? std::string("annonymous-") + std::to_string(inner_scope.storyboard_elements.size())
                       : name,
          make<U>(node, inner_scope))
      };

      if (not cdr(result))
      {
        std::stringstream ss {};
        ss << "detected redefinition of StoryboardElement named \'" << name << "\'";
        throw SyntaxError { ss.str() };
      }
      else
      {
        static_cast<T&>(*this).push_back(car(result)->second);
        return car(result)->second;
      }
    }

  public:
    /* ==== States and Transitions of StoryboardElements =======================
     *
     * See https://releases.asam.net/OpenSCENARIO/1.0.0/ASAM_OpenSCENARIO_BS-1-2_User-Guide_V1-0-0.html#_states_and_transitions_of_storyboardelements
     *
     * ====================================================================== */
    auto evaluate()
    {
      if (stopTriggered())
      {
        override();
      }

      std::cout << (indent++)
                << "Evaluating "
                << cyan
                << "\""
                << static_cast<const T&>(*this).name
                << "\""
                << reset
                << " [" << state << "] "
                << std::endl;

      BOOST_SCOPE_EXIT_ALL()
      {
        --indent;
      };

      switch (currentState().template as<StoryboardElementState>(__FILE__, __LINE__))
      {
      /* ---- StandBy ----------------------------------------------------------
       *
       * This is the default initialization state of a StoryboardElement. When
       * it is in this state, the runtime instantiation of the StoryboardElement
       * is ready to execute once given a startTrigger. A runtime instantiation
       * of any StoryboardElement is created once its parent element is in the
       * standbyState. From the standbyState, the Story element instantaneously
       * transitions into the runningState.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::standbyState:

        if (not ready())
        {
          return state;
        }
        else
        {
          std::cout << indent
                    << typeid(T).name()
                    << "::evaluate [" << state << " => " << start_transition << "]"
                    << std::endl;

          return state = start_transition;
        }

      /* ---- Start ------------------------------------------------------------
       *
       * The startTransition symbolizes that the execution of the runtime
       * instantiation is now starting. The startTransition can be used in
       * conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::startTransition:

        static_cast<T&>(*this).start();

        ++execution_count;

        std::cout << indent
                  << typeid(T).name()
                  << "::evaluate [" << state << " => " << running_state << "]"
                  << std::endl;

        return state = running_state;

      /* ---- Running ----------------------------------------------------------
       *
       * The runningState symbolizes that the execution of the runtime
       * instantiation is now ongoing and has not yet accomplished its goal.
       *
       * The concept of accomplishing a goal varies depending on the type of
       * StoryboardElement under consideration:
       *
       * Action
       *   An Action's goal is a function of the Action type and cannot be
       *   generalized. Accomplishing an Action's goal will involve meeting some
       *   arbitrary prerequisites related with the Action type (for example, a
       *   SpeedAction accomplishes its goal when the considered Entity is
       *   travelling at the prescribed speed). If an Action is acting on an
       *   EntitySelection, all instances of Entity within the selection have to
       *   complete in order to reach the completeState of the Action.
       *
       * Event
       *   An Event's goal is accomplished when all its Actions are in the
       *   completeState.
       *
       * Maneuver
       *   A Maneuver's goal is accomplished when all its Events are in the
       *   completeState.
       *
       * ManeuverGroup
       *   A ManeuverGroup's goal is accomplished when all its Maneuvers are in
       *   the completeState.
       *
       * Act
       *   An Act's goal is accomplished when all its ManeuverGroups are in the
       *   completeState.
       *
       * Story
       *   A Story's goal is accomplished when all its Acts are in the
       *   completeState.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::runningState:

        static_cast<T&>(*this).run();

        if (not accomplished())
        {
          return state;
        }
        else
        {
          std::cout << indent
                    << typeid(T).name()
                    << "::evaluate [" << state << " => " << end_transition << "]"
                    << std::endl;

          return state = end_transition;
        }

      /* ---- End --------------------------------------------------------------
       *
       * The endTransition occurs when the runtime instantiation of the
       * StoryboardElement accomplishes its goal. Once the endTransition occurs,
       * a check for completeness is made. A positive outcome moves the state
       * machine to the completeState, whereas a negative outcome moves the
       * state machine to the standbyState. The endTransition can be used in
       * conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::endTransition:

        if (execution_count < maximum_execution_count) // check for completeness
        {
          return state = standby_state;
        }
        else
        {
          return state = complete_state;
        }

      /* ---- Complete ---------------------------------------------------------
       *
       * The completeState signals that the runtime instantiation of the
       * StoryboardElement cannot reach a running state without external
       * interference. If the affected runtime instantiation of the
       * StoryboardElement is defined with a maximumExecutionCount, to be
       * complete implies that there are no more executions left to run, or a
       * stopTransition has occurred.
       *
       * Checking for completeness involves verifying if the given runtime
       * instantiation of the StoryboardElement still has executions left upon
       * finishing the runningState. This check returns false if there are
       * executions left. This check returns true if there are no executions
       * left, or if the maximumExecutionCount is not defined in the
       * StoryboardElement.
       *
       * Resetting the completeState can only be achieved externally by the
       * parent StoryboardElement whose child is in the completeState. This may
       * only occur if the parent initiates a new execution.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::completeState:

        return state;

      /* ---- Skip -------------------------------------------------------------
       *
       * Transition marking the moment an element is asked to move to the
       * runningState but is instead skipped so it remains in the standbyState
       * (only for Event instances). The skipTransition can be used in
       * conditions to trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      case StoryboardElementState::skipTransition:

        return state;

      /* ---- Stop -------------------------------------------------------------
       *
       * The stopTransition marks the reception of a stopTrigger or the
       * storyboard element is overridden (applicable for Event and Action).
       * This implies that the stopTransition cannot be reached other than with
       * an external intervention to the runtime instantiation of the
       * StoryboardElement.
       *
       * When a runtime instantiation of a StoryboardElement goes through a
       * stopTransition, all of its child elements are also forced to go through
       * the same transition. The stopTransition can be used in conditions to
       * trigger based on this transition.
       *
       * -------------------------------------------------------------------- */
      default:
      case StoryboardElementState::stopTransition:

        if (not accomplished())
        {
          static_cast<T&>(*this).stop();

          return state;
        }
        else
        {
          std::cout << indent
                    << typeid(T).name()
                    << "::stop [" << state << " => " << complete_state << "]"
                    << std::endl;

          return state = complete_state;
        }
      }
    }
  };
}}  // namespace scenario_runner::syntax

#endif  // SCENARIO_RUNNER__SYNTAX__STORYBOARD_ELEMENT
