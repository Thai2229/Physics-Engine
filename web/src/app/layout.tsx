import type { Metadata } from 'next';
import './globals.css';

export const metadata: Metadata = {
  title: 'Physics Engine Demo',
  description: 'A C++/WebAssembly 2D rigid-body physics engine: editable robotic arm and scene gallery.',
};

export default function RootLayout({ children }: { children: React.ReactNode }) {
  return (
    <html lang="en">
      <body>{children}</body>
    </html>
  );
}
