# GitHub Pages Deployment Guide for Physical AI & Humanoid Robotics Book

Ye guide aapko step-by-step instructions provide karegi ke GitHub Pages par aapki Roman Urdu book deploy kaise karni hai.

## Prerequisites

- Git installed aapke system par
- GitHub account
- Node.js aur npm installed
- Docusaurus project locally setup

## Step 1: Initialize Git Repository

```bash
# Aapke project directory mein navigate karein
cd C:\ai-book\ai-book

# Git repository initialize karein
git init

# Initial files add karein
git add .

# Initial commit karein
git commit -m "Initial commit: Physical AI & Humanoid Robotics book"
```

## Step 2: Create GitHub Repository

1. GitHub.com par jaayein
2. New repository create karein
3. Repository name: `physical-ai-humanoid-robotics`
4. Description: "Roman Urdu guide to Physical AI & Humanoid Robotics"
5. Public ya Private choose karein (Public recommended for GitHub Pages)
6. "Initialize this repository with a README" ko unchecked rakhein
7. Create repository button click karein

## Step 3: Connect Local Repository to GitHub

```bash
# GitHub repository ko remote add karein
git remote add origin https://github.com/your-github-username/physical-ai-humanoid-robotics.git

# Replace 'your-github-username' ko apne actual GitHub username se
```

## Step 4: Configure Docusaurus for GitHub Pages

Make sure aapka `docusaurus.config.ts` file ye settings contain kar raha ho:

```javascript
const config = {
  // ...
  url: 'https://your-github-username.github.io',
  baseUrl: '/physical-ai-humanoid-robotics/',
  organizationName: 'your-github-username', // Usually your GitHub org/user name.
  projectName: 'physical-ai-humanoid-robotics', // Usually your repo name.
  // ...
};
```

## Step 5: Push Code to GitHub

```bash
# Code ko GitHub par push karein
git branch -M main
git push -u origin main
```

## Step 6: Deploy to GitHub Pages

```bash
# Docusaurus build karein aur deploy karein
npm run deploy
```

Ye command automatically:

1. Production build create karta hai
2. `gh-pages` branch create/update karta hai
3. Built files ko gh-pages branch par push karta hai

## Step 7: Enable GitHub Pages

1. GitHub repository par jaayein
2. Settings tab click karein
3. Left sidebar mein "Pages" click karein
4. Source section mein:
   - Branch: `gh-pages` select karein
   - `/ (root)` ya `/docs` select karein (depending on your setup)
5. Save button click karein

## Step 8: Access Your Published Book

Aapki book accessible hogi is URL par:
```
https://your-github-username.github.io/physical-ai-humanoid-robotics/
```

Replace `your-github-username` ko apne actual GitHub username se.

## Future Updates

Jab aap book mein changes karenge, sirf ye commands run karein:

```bash
# Changes commit karein
git add .
git commit -m "Update: [describe your changes]"

# GitHub par push karein
git push origin main

# Docusaurus deploy karein
npm run deploy
```

## Troubleshooting

Agar deployment mein issues aate hain:

1. Make sure `docusaurus.config.ts` mein sahi URLs aur base URLs hain
2. Check ke `package.json` mein ye script hai:
   ```json
   "scripts": {
     "deploy": "docusaurus deploy"
   }
   ```
3. Verify ke GitHub repository public hai (agar public access chahte hain)
4. Wait few minutes agar website load nahi ho rahi (sometimes takes 5-10 minutes)

## Custom Domain (Optional)

Agar aap custom domain use karna chahte hain:

1. Domain purchase karein
2. GitHub repository settings mein jaayein
3. Pages section mein custom domain add karein
4. DNS settings update karein apne domain registrar ke through

Aapki Physical AI & Humanoid Robotics book ab successfully GitHub Pages par deploy ho chuki hai!