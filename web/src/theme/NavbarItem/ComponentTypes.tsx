/**
 * Custom Navbar Component Types Registration
 *
 * Maps custom navbar item types to their respective components
 * Extends Docusaurus default navbar items with custom auth component
 */

import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import AuthNavbarItem from '@site/src/components/AuthNavbarItem';

export default {
  ...ComponentTypes,
  'custom-authNavbarItem': AuthNavbarItem,
};
