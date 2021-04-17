/*
 * Team 4909, Bionics
 * Billerica Memorial High School
 *
 * Copyright:
 *   2021 Bionics
 *
 * License:
 *   MIT: https://opensource.org/licenses/MIT
 *   See the LICENSE file in the project's top-level directory for details.
 */

package frc.bionic;

public class UserInterfaceElement<T>
{
  private T element;

  public UserInterfaceElement(T element)
  {
    this.element = element;
  }

  public T get()
  {
    return element;
  }
}