/**
 * Custom Navbar Component Types Registration
 *
 * Maps custom navbar item types to their respective components
 * Extends Docusaurus default navbar items with custom auth and language components
 */

import ComponentTypes from '@theme-original/NavbarItem/ComponentTypes';
import AuthNavbarItem from '@site/src/components/AuthNavbarItem';
import LanguageToggle from '@site/src/components/LanguageToggle';

export default {
  ...ComponentTypes,
  'custom-authNavbarItem': AuthNavbarItem,
  'custom-languageToggle': LanguageToggle,
};
